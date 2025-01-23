/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ISAM2.cpp
 * @brief   Incremental update functionality (ISAM2) for BayesTree, with fluid
 * relinearization.
 * @author  Michael Kaess, Richard Roberts, Frank Dellaert
 */

// the following headers are necesarry to avoid incomplete class errros on nvcc
#include <boost/utility/in_place_factory.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

#include <gtsam_points/optimizers/isam2_ext_impl.hpp>
#include <gtsam/nonlinear/ISAM2Result.h>

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>
#include <gtsam/inference/BayesTree-inst.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/dogleg_optimizer_ext_impl.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

#include <algorithm>
#include <map>
#include <utility>

using namespace std;

namespace gtsam {
// Instantiate base class
template class BayesTree<ISAM2Clique>;
}  // namespace gtsam

namespace gtsam_points {

/* ************************************************************************* */
ISAM2Ext::ISAM2Ext(const ISAM2Params& params) : params_(params), update_count_(0) {
  if (params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ = boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;

  linearization_hook.reset(new LinearizationHook());
}

/* ************************************************************************* */
ISAM2Ext::ISAM2Ext() : update_count_(0) {
  if (params_.optimizationParams.type() == typeid(ISAM2DoglegParams))
    doglegDelta_ = boost::get<ISAM2DoglegParams>(params_.optimizationParams).initialDelta;

  linearization_hook.reset(new LinearizationHook());
}

/* ************************************************************************* */
ISAM2Ext::~ISAM2Ext() {}

/* ************************************************************************* */
bool ISAM2Ext::equals(const ISAM2Ext& other, double tol) const {
  return Base::equals(other, tol) && theta_.equals(other.theta_, tol) && variableIndex_.equals(other.variableIndex_, tol) &&
         nonlinearFactors_.equals(other.nonlinearFactors_, tol) && fixedVariables_ == other.fixedVariables_;
}

/* ************************************************************************* */
GaussianFactorGraph
ISAM2Ext::relinearizeAffectedFactors(const ISAM2UpdateParams& updateParams, const FastList<Key>& affectedKeys, const KeySet& relinKeys) {
  gttic(relinearizeAffectedFactors);
  FactorIndexSet candidates = UpdateImpl::GetAffectedFactors(affectedKeys, variableIndex_);

  gttic(affectedKeysSet);
  // for fast lookup below
  KeySet affectedKeysSet;
  affectedKeysSet.insert(affectedKeys.begin(), affectedKeys.end());
  gttoc(affectedKeysSet);

  gttic(check_candidates_and_linearize);
  GaussianFactorGraph linearized;

  linearization_hook->clear();
  std::vector<FactorIndex> gpu_factor_indices;

  for (const FactorIndex idx : candidates) {
    bool inside = true;
    bool useCachedLinear = params_.cacheLinearizedFactors;
    for (Key key : nonlinearFactors_[idx]->keys()) {
      if (affectedKeysSet.find(key) == affectedKeysSet.end()) {
        inside = false;
        break;
      }
      if (useCachedLinear && relinKeys.find(key) != relinKeys.end()) useCachedLinear = false;
    }
    if (inside) {
      if (useCachedLinear) {
        linearized.push_back(linearFactors_[idx]);
      } else {
        // GPU factor
        if (linearization_hook->add(nonlinearFactors_[idx])) {
          gpu_factor_indices.push_back(idx);
        }
        // non GPU factor
        else {
          auto linearFactor = nonlinearFactors_[idx]->linearize(theta_);
          linearized.push_back(linearFactor);
          if (params_.cacheLinearizedFactors) {
            linearFactors_[idx] = linearFactor;
          }
        }
      }
    }
  }
  gttoc(check_candidates_and_linearize);

  auto linear_factors_gpu = linearization_hook->calc_linear_factors(theta_);
  for (int i = 0; i < linear_factors_gpu.size(); i++) {
    const auto& linearFactor = linear_factors_gpu[i];
    const FactorIndex idx = gpu_factor_indices[i];

    linearized.push_back(linearFactor);
    if (params_.cacheLinearizedFactors) {
      linearFactors_[idx] = linearFactor;
    }
  }

  return linearized;
}

/* ************************************************************************* */
void ISAM2Ext::recalculate(const ISAM2UpdateParams& updateParams, const KeySet& relinKeys, ISAM2Result* result) {
  gttic(recalculate);
  UpdateImpl::LogRecalculateKeys(*result);

  if (!result->markedKeys.empty() || !result->observedKeys.empty()) {
    // Remove top of Bayes tree and convert to a factor graph:
    // (a) For each affected variable, remove the corresponding clique and all
    // parents up to the root. (b) Store orphaned sub-trees \BayesTree_{O} of
    // removed cliques.
    GaussianBayesNet affectedBayesNet;
    Cliques orphans;
    this->removeTop(KeyVector(result->markedKeys.begin(), result->markedKeys.end()), &affectedBayesNet, &orphans);

    // FactorGraph<GaussianFactor> factors(affectedBayesNet);
    // bug was here: we cannot reuse the original factors, because then the
    // cached factors get messed up [all the necessary data is actually
    // contained in the affectedBayesNet, including what was passed in from the
    // boundaries, so this would be correct; however, in the process we also
    // generate new cached_ entries that will be wrong (ie. they don't contain
    // what would be passed up at a certain point if batch elimination was done,
    // but that's what we need); we could choose not to update cached_ from
    // here, but then the new information (and potentially different variable
    // ordering) is not reflected in the cached_ values which again will be
    // wrong] so instead we have to retrieve the original linearized factors AND
    // add the cached factors from the boundary

    // ordering provides all keys in conditionals, there cannot be others
    // because path to root included
    gttic(affectedKeys);
    FastList<Key> affectedKeys;
    for (const auto& conditional : affectedBayesNet)
      affectedKeys.insert(affectedKeys.end(), conditional->beginFrontals(), conditional->endFrontals());
    gttoc(affectedKeys);

    KeySet affectedKeysSet;
    static const double kBatchThreshold = 0.65;
    if (affectedKeys.size() >= theta_.size() * kBatchThreshold) {
      // Do a batch step - reorder and relinearize all variables
      recalculateBatch(updateParams, &affectedKeysSet, result);
    } else {
      recalculateIncremental(updateParams, relinKeys, affectedKeys, &affectedKeysSet, &orphans, result);
    }

    // Root clique variables for detailed results
    if (result->detail && params_.enableDetailedResults) {
      for (const auto& root : roots_)
        for (Key var : *root->conditional()) result->detail->variableStatus[var].inRootClique = true;
    }

    // Update replaced keys mask (accumulates until back-substitution happens)
    deltaReplacedMask_.insert(affectedKeysSet.begin(), affectedKeysSet.end());
  }
}

/* ************************************************************************* */
void ISAM2Ext::recalculateBatch(const ISAM2UpdateParams& updateParams, KeySet* affectedKeysSet, ISAM2Result* result) {
  gttic(recalculateBatch);

  gttic(add_keys);
  br::copy(variableIndex_ | br::map_keys, std::inserter(*affectedKeysSet, affectedKeysSet->end()));

  // Removed unused keys:
  VariableIndex affectedFactorsVarIndex = variableIndex_;

  affectedFactorsVarIndex.removeUnusedVariables(result->unusedKeys.begin(), result->unusedKeys.end());

  for (const Key key : result->unusedKeys) {
    affectedKeysSet->erase(key);
  }
  gttoc(add_keys);

  gttic(ordering);
  Ordering order;
  if (updateParams.constrainedKeys) {
    order = Ordering::ColamdConstrained(affectedFactorsVarIndex, *updateParams.constrainedKeys);
  } else {
    if (theta_.size() > result->observedKeys.size()) {
      // Only if some variables are unconstrained
      FastMap<Key, int> constraintGroups;
      for (Key var : result->observedKeys) constraintGroups[var] = 1;
      order = Ordering::ColamdConstrained(affectedFactorsVarIndex, constraintGroups);
    } else {
      order = Ordering::Colamd(affectedFactorsVarIndex);
    }
  }
  gttoc(ordering);

  gttic(linearize);
  // issue linearization of GPU factors
  linearization_hook->clear();
  linearization_hook->add(nonlinearFactors_);
  linearization_hook->linearize(theta_);

  auto linearized = nonlinearFactors_.linearize(theta_);
  if (params_.cacheLinearizedFactors) linearFactors_ = *linearized;
  gttoc(linearize);

  gttic(eliminate);
  ISAM2BayesTree::shared_ptr bayesTree =
    ISAM2JunctionTree(GaussianEliminationTree(*linearized, affectedFactorsVarIndex, order)).eliminate(params_.getEliminationFunction()).first;
  gttoc(eliminate);

  gttic(insert);
  roots_.clear();
  roots_.insert(roots_.end(), bayesTree->roots().begin(), bayesTree->roots().end());
  nodes_.clear();
  nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
  gttoc(insert);

  result->variablesReeliminated = affectedKeysSet->size();
  result->factorsRecalculated = nonlinearFactors_.size();

  // Reeliminated keys for detailed results
  if (params_.enableDetailedResults) {
    for (Key key : theta_.keys()) {
      result->detail->variableStatus[key].isReeliminated = true;
    }
  }
}

/* ************************************************************************* */
void ISAM2Ext::recalculateIncremental(
  const ISAM2UpdateParams& updateParams,
  const KeySet& relinKeys,
  const FastList<Key>& affectedKeys,
  KeySet* affectedKeysSet,
  Cliques* orphans,
  ISAM2Result* result) {
  gttic(recalculateIncremental);
  const bool debug = ISDEBUG("ISAM2 recalculate");

  // 2. Add the new factors \Factors' into the resulting factor graph
  FastList<Key> affectedAndNewKeys;
  affectedAndNewKeys.insert(affectedAndNewKeys.end(), affectedKeys.begin(), affectedKeys.end());
  affectedAndNewKeys.insert(affectedAndNewKeys.end(), result->observedKeys.begin(), result->observedKeys.end());
  GaussianFactorGraph factors = relinearizeAffectedFactors(updateParams, affectedAndNewKeys, relinKeys);

  if (debug) {
    factors.print("Relinearized factors: ");
    std::cout << "Affected keys: ";
    for (const Key key : affectedKeys) {
      std::cout << key << " ";
    }
    std::cout << std::endl;
  }

  // Reeliminated keys for detailed results
  if (params_.enableDetailedResults) {
    for (Key key : affectedAndNewKeys) {
      result->detail->variableStatus[key].isReeliminated = true;
    }
  }

  result->variablesReeliminated = affectedAndNewKeys.size();
  result->factorsRecalculated = factors.size();

  gttic(cached);
  // Add the cached intermediate results from the boundary of the orphans...
  GaussianFactorGraph cachedBoundary = UpdateImpl::GetCachedBoundaryFactors(*orphans);
  if (debug) cachedBoundary.print("Boundary factors: ");
  factors.push_back(cachedBoundary);
  gttoc(cached);

  gttic(orphans);
  // Add the orphaned subtrees
  for (const auto& orphan : *orphans) factors += boost::make_shared<BayesTreeOrphanWrapper<ISAM2Ext::Clique> >(orphan);
  gttoc(orphans);

  // 3. Re-order and eliminate the factor graph into a Bayes net (Algorithm
  // [alg:eliminate]), and re-assemble into a new Bayes tree (Algorithm
  // [alg:BayesTree])

  gttic(reorder_and_eliminate);

  gttic(list_to_set);
  // create a partial reordering for the new and contaminated factors
  // result->markedKeys are passed in: those variables will be forced to the
  // end in the ordering
  affectedKeysSet->insert(result->markedKeys.begin(), result->markedKeys.end());
  affectedKeysSet->insert(affectedKeys.begin(), affectedKeys.end());
  gttoc(list_to_set);

  VariableIndex affectedFactorsVarIndex(factors);

  gttic(ordering_constraints);
  // Create ordering constraints
  FastMap<Key, int> constraintGroups;
  if (updateParams.constrainedKeys) {
    constraintGroups = *updateParams.constrainedKeys;
  } else {
    constraintGroups = FastMap<Key, int>();
    const int group = result->observedKeys.size() < affectedFactorsVarIndex.size() ? 1 : 0;
    for (Key var : result->observedKeys) constraintGroups.insert(std::make_pair(var, group));
  }

  // Remove unaffected keys from the constraints
  for (FastMap<Key, int>::iterator iter = constraintGroups.begin(); iter != constraintGroups.end();
       /*Incremented in loop ++iter*/) {
    if (result->unusedKeys.exists(iter->first) || !affectedKeysSet->exists(iter->first))
      constraintGroups.erase(iter++);
    else
      ++iter;
  }
  gttoc(ordering_constraints);

  // Generate ordering
  gttic(Ordering);
  const Ordering ordering = Ordering::ColamdConstrained(affectedFactorsVarIndex, constraintGroups);
  gttoc(Ordering);

  // Do elimination
  GaussianEliminationTree etree(factors, affectedFactorsVarIndex, ordering);
  auto bayesTree = ISAM2JunctionTree(etree).eliminate(params_.getEliminationFunction()).first;
  gttoc(reorder_and_eliminate);

  gttic(reassemble);
  roots_.insert(roots_.end(), bayesTree->roots().begin(), bayesTree->roots().end());
  nodes_.insert(bayesTree->nodes().begin(), bayesTree->nodes().end());
  gttoc(reassemble);

  // 4. The orphans have already been inserted during elimination
}

/* ************************************************************************* */
void ISAM2Ext::addVariables(const Values& newTheta, ISAM2Result::DetailedResults* detail) {
  gttic(addNewVariables);

  theta_.insert(newTheta);
  if (ISDEBUG("ISAM2 AddVariables")) newTheta.print("The new variables are: ");
  // Add zeros into the VectorValues
  delta_.insert(newTheta.zeroVectors());
  deltaNewton_.insert(newTheta.zeroVectors());
  RgProd_.insert(newTheta.zeroVectors());

  // New keys for detailed results
  if (detail && params_.enableDetailedResults) {
    for (Key key : newTheta.keys()) {
      detail->variableStatus[key].isNew = true;
    }
  }
}

/* ************************************************************************* */
void ISAM2Ext::removeVariables(const KeySet& unusedKeys) {
  gttic(removeVariables);

  variableIndex_.removeUnusedVariables(unusedKeys.begin(), unusedKeys.end());
  for (Key key : unusedKeys) {
    delta_.erase(key);
    deltaNewton_.erase(key);
    RgProd_.erase(key);
    deltaReplacedMask_.erase(key);
    Base::nodes_.unsafe_erase(key);
    theta_.erase(key);
    fixedVariables_.erase(key);
  }
}

/* ************************************************************************* */
ISAM2ResultExt ISAM2Ext::update(
  const NonlinearFactorGraph& newFactors,
  const Values& newTheta,
  const FactorIndices& removeFactorIndices,
  const boost::optional<FastMap<Key, int> >& constrainedKeys,
  const boost::optional<FastList<Key> >& noRelinKeys,
  const boost::optional<FastList<Key> >& extraReelimKeys,
  bool force_relinearize) {
  ISAM2UpdateParams params;
  params.constrainedKeys = constrainedKeys;
  params.extraReelimKeys = extraReelimKeys;
  params.force_relinearize = force_relinearize;
  params.noRelinKeys = noRelinKeys;
  params.removeFactorIndices = removeFactorIndices;

  return update(newFactors, newTheta, params);
}

/* ************************************************************************* */
ISAM2ResultExt ISAM2Ext::update(const NonlinearFactorGraph& newFactors, const Values& newTheta, const ISAM2UpdateParams& updateParams) {
  gttic(ISAM2_update);
  optimization_start_time = std::chrono::high_resolution_clock::now();

  this->update_count_ += 1;
  UpdateImpl::LogStartingUpdate(newFactors, *this);
  ISAM2ResultExt result(params_.enableDetailedResults);

  UpdateImpl update(params_, updateParams);

  linearization_hook->clear_counts();

  // Update delta if we need it to check relinearization later
  if (update.relinarizationNeeded(update_count_)) updateDelta(updateParams.forceFullSolve);

  // 1. Add any new factors \Factors:=\Factors\cup\Factors'.
  update.pushBackFactors(newFactors, &nonlinearFactors_, &linearFactors_, &variableIndex_, &result.newFactorsIndices, &result.keysWithRemovedFactors);
  update.computeUnusedKeys(newFactors, variableIndex_, result.keysWithRemovedFactors, &result.unusedKeys);

  // 2. Initialize any new variables \Theta_{new} and add
  // \Theta:=\Theta\cup\Theta_{new}.
  addVariables(newTheta, result.details());
  if (params_.evaluateNonlinearError) {
    auto estimate = calculateEstimate();
    linearization_hook->clear();
    linearization_hook->add(nonlinearFactors_);
    linearization_hook->linearize(theta_);
    linearization_hook->error(estimate);

    update.error(nonlinearFactors_, estimate, &result.errorBefore);
  }

  // 3. Mark linear update
  update.gatherInvolvedKeys(newFactors, nonlinearFactors_, result.keysWithRemovedFactors, &result.markedKeys);
  update.updateKeys(result.markedKeys, &result);

  KeySet relinKeys;
  result.variablesRelinearized = 0;
  if (update.relinarizationNeeded(update_count_)) {
    // 4. Mark keys in \Delta above threshold \beta:
    relinKeys = update.gatherRelinearizeKeys(roots_, delta_, fixedVariables_, &result.markedKeys);
    update.recordRelinearizeDetail(relinKeys, result.details());
    if (!relinKeys.empty()) {
      // 5. Mark cliques that involve marked variables \Theta_{J} and ancestors.
      update.findFluid(roots_, relinKeys, &result.markedKeys, result.details());
      // 6. Update linearization point for marked variables:
      // \Theta_{J}:=\Theta_{J}+\Delta_{J}.
      UpdateImpl::ExpmapMasked(delta_, relinKeys, &theta_);
    }
    result.variablesRelinearized = result.markedKeys.size();
  }

  // 7. Linearize new factors
  linearization_hook->clear();
  linearization_hook->add(newFactors);
  linearization_hook->linearize(theta_);

  update.linearizeNewFactors(newFactors, theta_, nonlinearFactors_.size(), result.newFactorsIndices, &linearFactors_);
  update.augmentVariableIndex(newFactors, result.newFactorsIndices, &variableIndex_);

  // 8. Redo top of Bayes tree and update data structures
  recalculate(updateParams, relinKeys, &result);
  if (!result.unusedKeys.empty()) removeVariables(result.unusedKeys);
  result.cliques = this->nodes().size();

  if (params_.evaluateNonlinearError) {
    auto estimate = calculateEstimate();
    linearization_hook->clear();
    linearization_hook->add(nonlinearFactors_);
    linearization_hook->linearize(theta_);
    linearization_hook->error(estimate);

    update.error(nonlinearFactors_, estimate, &result.errorAfter);
  }

  result.update_count = update_count_;
  result.delta = delta_.norm();
  result.gpu_evaluation_count = linearization_hook->evaluation_count();
  result.gpu_linearization_count = linearization_hook->linearization_count();
  result.elapsed_time =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - optimization_start_time).count() / 1e9;

  result.num_factors = this->nonlinearFactors_.size();
  result.num_values = this->theta_.size();

  return result;
}

/* ************************************************************************* */
void ISAM2Ext::marginalizeLeaves(
  const FastList<Key>& leafKeysList,
  boost::optional<FactorIndices&> marginalFactorsIndices,
  boost::optional<FactorIndices&> deletedFactorsIndices) {
  // Convert to ordered set
  KeySet leafKeys(leafKeysList.begin(), leafKeysList.end());

  // Keep track of marginal factors - map from clique to the marginal factors
  // that should be incorporated into it, passed up from it's children.
  //  multimap<sharedClique, GaussianFactor::shared_ptr> marginalFactors;
  map<Key, vector<GaussianFactor::shared_ptr> > marginalFactors;

  // Keep track of variables removed in subtrees
  KeySet leafKeysRemoved;

  // Keep track of factors that get summarized by removing cliques
  FactorIndexSet factorIndicesToRemove;

  // Remove the subtree and throw away the cliques
  auto trackingRemoveSubtree = [&](const sharedClique& subtreeRoot) {
    const Cliques removedCliques = this->removeSubtree(subtreeRoot);
    for (const sharedClique& removedClique : removedCliques) {
      auto cg = removedClique->conditional();
      marginalFactors.erase(cg->front());
      leafKeysRemoved.insert(cg->beginFrontals(), cg->endFrontals());
      for (Key frontal : cg->frontals()) {
        // Add to factors to remove
        const auto& involved = variableIndex_[frontal];
        factorIndicesToRemove.insert(involved.begin(), involved.end());
#if !defined(NDEBUG)
        // Check for non-leaf keys
        if (!leafKeys.exists(frontal))
          throw std::runtime_error(
            "Requesting to marginalize variables that are not leaves, "
            "the ISAM2 object is now in an inconsistent state so should "
            "no longer be used.");
#endif
      }
    }
    return removedCliques;
  };

  // Remove each variable and its subtrees
  for (Key j : leafKeys) {
    if (!leafKeysRemoved.exists(j)) {  // If the index was not already removed
                                       // by removing another subtree

      // Traverse up the tree to find the root of the marginalized subtree
      sharedClique clique = nodes_[j];
      while (!clique->parent_._empty()) {
        // Check if parent contains a marginalized leaf variable.  Only need to
        // check the first variable because it is the closest to the leaves.
        sharedClique parent = clique->parent();
        if (leafKeys.exists(parent->conditional()->front()))
          clique = parent;
        else
          break;
      }

      // See if we should remove the whole clique
      bool marginalizeEntireClique = true;
      for (Key frontal : clique->conditional()->frontals()) {
        if (!leafKeys.exists(frontal)) {
          marginalizeEntireClique = false;
          break;
        }
      }

      // Remove either the whole clique or part of it
      if (marginalizeEntireClique) {
        // Remove the whole clique and its subtree, and keep the marginal
        // factor.
        auto marginalFactor = clique->cachedFactor();
        // We do not need the marginal factors associated with this clique
        // because their information is already incorporated in the new
        // marginal factor.  So, now associate this marginal factor with the
        // parent of this clique.
        marginalFactors[clique->parent()->conditional()->front()].push_back(marginalFactor);
        // Now remove this clique and its subtree - all of its marginal
        // information has been stored in marginalFactors.
        trackingRemoveSubtree(clique);
      } else {
        // Reeliminate the current clique and the marginals from its children,
        // then keep only the marginal on the non-marginalized variables.  We
        // get the childrens' marginals from any existing children, plus
        // the marginals from the marginalFactors multimap, which come from any
        // subtrees already marginalized out.

        // Add child marginals and remove marginalized subtrees
        GaussianFactorGraph graph;
        KeySet factorsInSubtreeRoot;
        Cliques subtreesToRemove;
        for (const sharedClique& child : clique->children) {
          // Remove subtree if child depends on any marginalized keys
          for (Key parent : child->conditional()->parents()) {
            if (leafKeys.exists(parent)) {
              subtreesToRemove.push_back(child);
              graph.push_back(child->cachedFactor());  // Add child marginal
              break;
            }
          }
        }
        Cliques childrenRemoved;
        for (const sharedClique& subtree : subtreesToRemove) {
          const Cliques removed = trackingRemoveSubtree(subtree);
          childrenRemoved.insert(childrenRemoved.end(), removed.begin(), removed.end());
        }

        // Add the factors that are pulled into the current clique by the
        // marginalized variables. These are the factors that involve
        // *marginalized* frontal variables in this clique but do not involve
        // frontal variables of any of its children.
        // TODO(dellaert): reuse cached linear factors
        KeySet factorsFromMarginalizedInClique_step1;
        for (Key frontal : clique->conditional()->frontals()) {
          if (leafKeys.exists(frontal)) factorsFromMarginalizedInClique_step1.insert(variableIndex_[frontal].begin(), variableIndex_[frontal].end());
        }
        // Remove any factors in subtrees that we're removing at this step
        for (const sharedClique& removedChild : childrenRemoved) {
          for (Key indexInClique : removedChild->conditional()->frontals()) {
            for (Key factorInvolving : variableIndex_[indexInClique]) {
              factorsFromMarginalizedInClique_step1.erase(factorInvolving);
            }
          }
        }
        // Create factor graph from factor indices
        linearization_hook->clear();
        for (const auto index : factorsFromMarginalizedInClique_step1) {
          if (!linearization_hook->add(nonlinearFactors_[index])) {
            // non GPU factors are linearized here
            graph.push_back(nonlinearFactors_[index]->linearize(theta_));
          }
        }

        // linearize GPU factors
        auto linear_factors_gpu = linearization_hook->calc_linear_factors(theta_);
        for (const auto& factor : linear_factors_gpu) {
          graph.push_back(factor);
        }

        // Reeliminate the linear graph to get the marginal and discard the
        // conditional
        auto cg = clique->conditional();
        const KeySet cliqueFrontals(cg->beginFrontals(), cg->endFrontals());
        KeyVector cliqueFrontalsToEliminate;
        std::set_intersection(
          cliqueFrontals.begin(),
          cliqueFrontals.end(),
          leafKeys.begin(),
          leafKeys.end(),
          std::back_inserter(cliqueFrontalsToEliminate));
        auto eliminationResult1 = params_.getEliminationFunction()(graph, Ordering(cliqueFrontalsToEliminate));

        // Add the resulting marginal
        if (eliminationResult1.second) marginalFactors[cg->front()].push_back(eliminationResult1.second);

        // Split the current clique
        // Find the position of the last leaf key in this clique
        DenseIndex nToRemove = 0;
        while (leafKeys.exists(cg->keys()[nToRemove])) ++nToRemove;

        // Make the clique's matrix appear as a subset
        const DenseIndex dimToRemove = cg->matrixObject().offset(nToRemove);
        cg->matrixObject().firstBlock() = nToRemove;
        cg->matrixObject().rowStart() = dimToRemove;

        // Change the keys in the clique
        KeyVector originalKeys;
        originalKeys.swap(cg->keys());
        cg->keys().assign(originalKeys.begin() + nToRemove, originalKeys.end());
        cg->nrFrontals() -= nToRemove;

        // Add to factorIndicesToRemove any factors involved in frontals of
        // current clique
        for (Key frontal : cliqueFrontalsToEliminate) {
          const auto& involved = variableIndex_[frontal];
          factorIndicesToRemove.insert(involved.begin(), involved.end());
        }

        // Add removed keys
        leafKeysRemoved.insert(cliqueFrontalsToEliminate.begin(), cliqueFrontalsToEliminate.end());
      }
    }
  }

  // At this point we have updated the BayesTree, now update the remaining iSAM2
  // data structures

  // Gather factors to add - the new marginal factors
  GaussianFactorGraph factorsToAdd;
  for (const auto& key_factors : marginalFactors) {
    for (const auto& factor : key_factors.second) {
      if (factor) {
        factorsToAdd.push_back(factor);
        if (marginalFactorsIndices) marginalFactorsIndices->push_back(nonlinearFactors_.size());
        nonlinearFactors_.push_back(boost::make_shared<LinearContainerFactor>(factor));
        if (params_.cacheLinearizedFactors) linearFactors_.push_back(factor);
        for (Key factorKey : *factor) {
          fixedVariables_.insert(factorKey);
        }
      }
    }
  }
  variableIndex_.augment(factorsToAdd);  // Augment the variable index

  // Remove the factors to remove that have been summarized in the newly-added
  // marginal factors
  NonlinearFactorGraph removedFactors;
  for (const auto index : factorIndicesToRemove) {
    removedFactors.push_back(nonlinearFactors_[index]);
    nonlinearFactors_.remove(index);
    if (params_.cacheLinearizedFactors) linearFactors_.remove(index);
  }
  variableIndex_.remove(factorIndicesToRemove.begin(), factorIndicesToRemove.end(), removedFactors);

  if (deletedFactorsIndices) deletedFactorsIndices->assign(factorIndicesToRemove.begin(), factorIndicesToRemove.end());

  // Remove the marginalized variables
  removeVariables(KeySet(leafKeys.begin(), leafKeys.end()));
}

/* ************************************************************************* */
// Marked const but actually changes mutable delta
void ISAM2Ext::updateDelta(bool forceFullSolve) const {
  gttic(updateDelta);
  if (params_.optimizationParams.type() == typeid(ISAM2GaussNewtonParams)) {
    // If using Gauss-Newton, update with wildfireThreshold
    const ISAM2GaussNewtonParams& gaussNewtonParams = boost::get<ISAM2GaussNewtonParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold = forceFullSolve ? 0.0 : gaussNewtonParams.wildfireThreshold;
    gttic(Wildfire_update);
    DeltaImpl::UpdateGaussNewtonDelta(roots_, deltaReplacedMask_, effectiveWildfireThreshold, &delta_);
    deltaReplacedMask_.clear();
    gttoc(Wildfire_update);

  } else if (params_.optimizationParams.type() == typeid(ISAM2DoglegParams)) {
    // If using Dogleg, do a Dogleg step
    const ISAM2DoglegParams& doglegParams = boost::get<ISAM2DoglegParams>(params_.optimizationParams);
    const double effectiveWildfireThreshold = forceFullSolve ? 0.0 : doglegParams.wildfireThreshold;

    // Do one Dogleg iteration
    gttic(Dogleg_Iterate);

    // Compute Newton's method step
    gttic(Wildfire_update);
    DeltaImpl::UpdateGaussNewtonDelta(roots_, deltaReplacedMask_, effectiveWildfireThreshold, &deltaNewton_);
    gttoc(Wildfire_update);

    // Compute steepest descent step
    const VectorValues gradAtZero = this->gradientAtZero();  // Compute gradient
    DeltaImpl::UpdateRgProd(roots_, deltaReplacedMask_, gradAtZero,
                            &RgProd_);                                                // Update RgProd
    const VectorValues dx_u = DeltaImpl::ComputeGradientSearch(gradAtZero, RgProd_);  // Compute gradient search point

    // Clear replaced keys mask because now we've updated deltaNewton_ and
    // RgProd_
    deltaReplacedMask_.clear();

    linearization_hook->clear();
    linearization_hook->add(nonlinearFactors_);
    linearization_hook->linearize(theta_);
    linearization_hook->error(theta_);

    double error0 = nonlinearFactors_.error(theta_);
    DoglegOptimizerImplExt::TrustRegionAdaptationMode adaptationMode =
      static_cast<DoglegOptimizerImplExt::TrustRegionAdaptationMode>(doglegParams.adaptationMode);

    // Compute dogleg point
    DoglegOptimizerImplExt::IterationResult doglegResult(DoglegOptimizerImplExt::Iterate(
      *doglegDelta_,
      adaptationMode,
      dx_u,
      deltaNewton_,
      *this,
      nonlinearFactors_,
      *linearization_hook,
      theta_,
      error0,
      doglegParams.verbose));
    gttoc(Dogleg_Iterate);

    gttic(Copy_dx_d);
    // Update Delta and linear step
    doglegDelta_ = doglegResult.delta;
    delta_ = doglegResult.dx_d;  // Copy the VectorValues containing with the linear solution
    gttoc(Copy_dx_d);
  } else {
    throw std::runtime_error("iSAM2: unknown ISAM2Params type");
  }
}

/* ************************************************************************* */
Values ISAM2Ext::calculateEstimate() const {
  gttic(ISAM2_calculateEstimate);
  const VectorValues& delta(getDelta());
  gttic(Expmap);
  return theta_.retract(delta);
  gttoc(Expmap);
}

/* ************************************************************************* */
const Value& ISAM2Ext::calculateEstimate(Key key) const {
  const Vector& delta = getDelta()[key];
  return *theta_.at(key).retract_(delta);
}

/* ************************************************************************* */
Values ISAM2Ext::calculateBestEstimate() const {
  updateDelta(true);  // Force full solve when updating delta_
  return theta_.retract(delta_);
}

/* ************************************************************************* */
Matrix ISAM2Ext::marginalCovariance(Key key) const {
  return marginalFactor(key, params_.getEliminationFunction())->information().inverse();
}

/* ************************************************************************* */
const VectorValues& ISAM2Ext::getDelta() const {
  if (!deltaReplacedMask_.empty()) updateDelta();
  return delta_;
}

/* ************************************************************************* */
double ISAM2Ext::error(const VectorValues& x) const {
  return GaussianFactorGraph(*this).error(x);
}

/* ************************************************************************* */
VectorValues ISAM2Ext::gradientAtZero() const {
  // Create result
  VectorValues g;

  // Sum up contributions for each clique
  for (const auto& root : this->roots()) root->addGradientAtZero(&g);

  return g;
}

}  // namespace gtsam_points
