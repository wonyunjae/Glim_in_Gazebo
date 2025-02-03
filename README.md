# Glim_in_Gazebo
Simulation Glim in Gazebo fortress (ignition 6)
`git clone --recursive https://github.com/wonyunjae/Glim_in_Gazebo.git`

**Glim-in-Gazebo-**

Simulation Glim in Gazebo fortress (ignition 6)

Glim in Gazebo Glim github : https://github.com/koide3/glim?tab=readme-ov-file Ros_gz github : https://github.com/gazebosim/ros_gz/tree/ros2 The environment : ubuntu 22.04, ROS2 humble, Gazebo Fortress (Ignition Gazebo 6)

**Glim in Gazebo Simulation**

https://github.com/louislelay/mobile-3d-lidar-sim

https://github.com/RobotecAI/RGLGazeboPlugin

https://github.com/gazebosim/ros_gz/tree/r

환경: Ubuntu 22.04, ROS2 Humble, Gazebo Fortress (Ignition Gazebo 6)

- Gazebo 환경 설치
    
    Gazebo는 각 환경에 맞게 설치하면 되지만, humble과 가장 호환성이 좋은 건 Gazebo Fortress 버전이기 때문에 Gazebo Fortress 버전을 설치한다.
    
    ```
    # gpg 키 추가
     sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
     sudo apt-get update
    
    # Ros-gz 설치
     sudo apt install ros-humble-ros-gz
    ```
    
    어떤 gpg키 를 추가하느냐에 따라 다른 Gazebo가 설치되고, 결국 호환성이 안맞는 오류가 발생해 재설치를 하게된다. 따라서 **Ignition Gazebo 6 apt** (Gazebo sim 7 X , Gazebo sim 9 X )만 설치되도록 유의하며 설치를 진행한다.
    
    ```
    # fortress 버전
    export GZ_VERSION=fortress
    
    # 워크스페이스에 소스 setup
    mkdir -p ~/ws/src
    cd ~/ws/src
    git clone https://github.com/gazebosim/ros_gz.git -b humble
    
    # 의존성 패키지 설치, rosdep으로 하면 편하다.
    cd ~/ws
    rosdep install -r --from-paths src -i -y --rosdistro humble
    
    # 빌드
    cd ~/ws
    source /opt/ros/<distro>/setup.bash
    colcon build --symlink-install
    ```
    
- Map 선정 및 센서 추가
    
    Glim을 위한 Gazebo 환경 설정이기에, 다양한 Gazebo worlds, models 중 slam하기에 알맞는 **tunnel.sdf** 을 선정했다.   tunnel.sdf → 1 camera, no LiDAR, no IMU
    
    Glim → 3D LiDAR & 9-axis IMU
    
    - tunnel.sdf 파일은 /usr/share/ignition/ignition-gazebo6/worlds 경로에 있다.
- Map 선정
    
    Glim을 위한 Gazebo 환경 설정이기에, 다양한 Gazebo worlds, models 중 slam하기에 알맞는 **tunnel.sdf** 을 선정했다.
    
    **<tunnel.sdf>** 
    
    tunnel.sdf → 1 camera, no LiDAR, no IMU
    
    Glim → 3D LiDAR & 9-axis IMU
    
    - tunnel.sdf 파일은 /usr/share/ignition/ignition-gazebo6/worlds 경로에 있다.
- 센서 추가
    
    **소스 setup**
    
    cd ~/ws/src git clone https://github.com/wonyunjae/Glim-in-Gazebo.git git checkout master
    
    **build**
    
    cd ~/ws colcon build --symlink-install
    
    ros2 launch ros_gz_sim_demos tunnel_glim_bridge.launch.py
    
    **another terminal**
    
    ros2 run glim_ros glim_rosnode
