# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r tunnel_glim.sdf'
        }.items(),
    )

    # RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'gpu_lidar_bridge.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/lidar/points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            'imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/vehicle/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/vehicle/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/vehicle/imu_tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/vehicle/pose', '/tf'),
            ('/model/vehicle/odometry', '/odom')
        ],
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./lidar/points.publisher.reliability': 'reliable',
            'qos_overrides./lidar/points.publisher.durability': 'volatile',
            'qos_overrides./lidar/points.publisher.history': 'keep_last',
            'qos_overrides./lidar/points.publisher.depth': 5,
            'qos_overrides.imu.publisher.reliability': 'reliable',
            'qos_overrides.imu.publisher.durability': 'volatile',
            'qos_overrides.imu.publisher.history': 'keep_last',
            'qos_overrides.imu.publisher.depth': 5
        }],
        output='screen'
    )
    
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'vehicle/base_link', 'vehicle/base_link/imu_sensor']
    )
    
     # LiDAR TF publisher 추가
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0.55', '0', '0.45', '0', '0', '0', 'vehicle/base_link', 'vehicle/base_link/lidar']
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        imu_tf,
        lidar_tf
    ])
