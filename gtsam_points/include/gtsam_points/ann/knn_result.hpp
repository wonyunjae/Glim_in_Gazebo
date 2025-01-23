// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <limits>
#include <iostream>

namespace gtsam_points {

/// @brief K-nearest neighbor search setting.
struct KnnSetting {
public:
  /// @brief Check if the result satisfies the early termination condition.
  template <typename Result>
  bool fulfilled(const Result& result) const {
    return result.worst_distance() < epsilon;
  }

public:
  double max_sq_dist = std::numeric_limits<double>::max();  ///< Maximum squared distance to search
  double epsilon = 0.0;                                     ///< Early termination threshold
};

/// @brief Identity transformation function. (use std::identity instead in C++20)
struct identity_transform {
  template <typename T>
  T operator()(const T& x) const {
    return x;
  }
};

/// @brief K-nearest neighbor search result container.
/// @tparam N   Number of neighbors to search. If N == -1, the number of neighbors is dynamicaly determined.
template <int N, typename IndexTransform = identity_transform>
struct KnnResult {
public:
  static constexpr size_t INVALID = std::numeric_limits<size_t>::max();

  /// @brief Constructor
  /// @param indices          Buffer to store indices (must be larger than k=max(N, num_neighbors))
  /// @param distances        Buffer to store distances (must be larger than k=max(N, num_neighbors))
  /// @param num_neighbors    Number of neighbors to search (must be -1 for static case N > 0)
  /// @param index_transform  Index transformation function (e.g., local point index -> global voxel + point index)
  explicit KnnResult(
    size_t* indices,
    double* distances,
    int num_neighbors = -1,
    const IndexTransform& index_transform = IndexTransform(),
    double max_sq_dist = std::numeric_limits<double>::max())
  : index_transform(index_transform),
    capacity(num_neighbors),
    num_found_neighbors(0),
    indices(indices),
    distances(distances) {
    if constexpr (N > 0) {
      if (num_neighbors >= 0) {
        std::cerr << "warning: Specifying dynamic num_neighbors=" << num_neighbors << " for a static KNN result container (N=" << N << ")"
                  << std::endl;
        abort();
      }
    } else {
      if (num_neighbors <= 0) {
        std::cerr << "error: Specifying invalid num_neighbors=" << num_neighbors << " for a dynamic KNN result container" << std::endl;
        abort();
      }
    }

    std::fill(this->indices, this->indices + buffer_size(), INVALID);
    std::fill(this->distances, this->distances + buffer_size(), max_sq_dist);
  }

  /// @brief  Buffer size (i.e., Maximum number of neighbors)
  size_t buffer_size() const {
    if constexpr (N > 0) {
      return N;
    } else {
      return capacity;
    }
  }

  /// @brief Number of found neighbors.
  size_t num_found() const { return num_found_neighbors; }

  /// @brief Worst distance in the result.
  double worst_distance() const { return distances[buffer_size() - 1]; }

  /// @brief  Push a pair of point index and distance to the result.
  void push(size_t index, double distance) {
    if (distance >= worst_distance()) {
      return;
    }

    if constexpr (N == 1) {
      indices[0] = index_transform(index);
      distances[0] = distance;
    } else {
      int insert_loc = std::min<int>(num_found_neighbors, buffer_size() - 1);
      for (; insert_loc > 0 && distance < distances[insert_loc - 1]; insert_loc--) {
        indices[insert_loc] = indices[insert_loc - 1];
        distances[insert_loc] = distances[insert_loc - 1];
      }

      indices[insert_loc] = index_transform(index);
      distances[insert_loc] = distance;
    }

    num_found_neighbors = std::min<int>(num_found_neighbors + 1, buffer_size());
  }

public:
  const IndexTransform& index_transform;
  const int capacity;       ///< Maximum number of neighbors to search
  int num_found_neighbors;  ///< Number of found neighbors
  size_t* indices;          ///< Indices of neighbors
  double* distances;        ///< Distances to neighbors
};

}  // namespace gtsam_points
