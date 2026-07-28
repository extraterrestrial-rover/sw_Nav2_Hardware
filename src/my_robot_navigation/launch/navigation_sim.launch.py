"""
navigation_sim.launch.py
─────────────────────────
Simulation test with REAL ZED2i camera + MOCK hardware (no ESP32 needed).

Architecture:
  - ZED2i camera:   provides live RGB, Depth, and visual odometry
  - mock hardware:  diff_drive_controller runs but sends no serial commands
  - EKF:            fuses ZED visual odometry → publishes odom→base_footprint TF
  - RTAB-Map:       LOCALIZATION mode — loads saved .db, publishes map→odom TF
  - map_server:     serves the saved .pgm file for Nav2 costmaps
  - Nav2:           full path planning stack — set 2D Nav Goals in RViz2!

Usage:
  Terminal 1 (ZED camera):
    source ~/zed2_ws/install/setup.bash
    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i \\
        publish_urdf:=false publish_tf:=false

  Terminal 2 (hardware + controllers, mock mode):
    source ~/nav2_hardware_ws/install/setup.bash
    ros2 launch my_robot_description hardware.launch.py

  Terminal 3 (this file — EKF + RTAB-Map localization + Nav2):
    source ~/nav2_hardware_ws/install/setup.bash
    ros2 launch my_robot_navigation navigation_sim.launch.py

  Terminal 4 (RViz2):
    source ~/nav2_hardware_ws/install/setup.bash
    rviz2 -d ~/nav2_hardware_ws/src/my_robot_description/rviz/nav2.rviz

In RViz2:
  1. Click "2D Pose Estimate" and click where the robot is on the map
  2. Click "2D Nav Goal" → click destination → watch Nav2 plan the path!
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav  = get_package_share_directory('my_robot_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2_params  = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    ekf_params   = os.path.join(pkg_nav, 'config', 'ekf_params.yaml')
    map_yaml     = os.path.join(pkg_nav, 'maps',   'rtabmap.yaml')

    # ── 1. EKF — fuses ZED visual odom → publishes /odometry/filtered + odom TF ─
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_params],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    # ── 2. Map Server — serves the pre-built .pgm map for Nav2 costmaps ──────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False,
        }],
    )

    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ── 3. RTAB-Map in LOCALIZATION mode ─────────────────────────────────────────
    # Loads the saved .db → re-localizes using live ZED2i feed.
    # Publishes the map→odom TF without growing the map.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom_info': False,
            'subscribe_odom': False,        # reads odom from TF (EKF output)
            'approx_sync': True,
            'wait_for_transform': 0.5,
            'use_sim_time': False,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            'qos_image': 2,
            'qos_camera_info': 2,
            'database_path': '~/.ros/rtabmap.db',
            # ── LOCALIZATION only — do NOT grow the map ─────────────────────────
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            # Grid map (published to /map topic — used as fallback)
            'Grid/FromDepth': 'true',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/RangeMax': '5.0',
            'Vis/MaxFeatures': '500',
        }],
        remappings=[
            ('rgb/image',       '/zed/zed_node/rgb/color/rect/image'),
            ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
            ('depth/image',     '/zed/zed_node/depth/depth_registered'),
        ],
    )

    # ── 4. Nav2 Navigation Stack ─────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
        }.items(),
    )

    return LaunchDescription([
        # EKF and map server start immediately
        ekf_node,
        map_server,
        map_lifecycle,
        # RTAB-Map after 2s so EKF TF is available
        TimerAction(period=2.0, actions=[rtabmap]),
        # Nav2 after 4s so map_server is active first
        TimerAction(period=4.0, actions=[nav2]),
    ])
