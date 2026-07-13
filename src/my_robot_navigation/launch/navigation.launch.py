import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_my_robot_navigation = get_package_share_directory('my_robot_navigation')

    # ── Launch Arguments ────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time (false for real hardware)')

    camera_model_arg = DeclareLaunchArgument(
        'camera_model', default_value='zed2i',
        description='ZED camera model')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = os.path.join(
        pkg_my_robot_navigation, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(
        pkg_my_robot_navigation, 'config', 'ekf_params.yaml')

    # ── 1. Nav2 Navigation Stack ────────────────────────────────────────────────
    #    Uses nav2_bringup's navigation_launch.py which starts:
    #      controller_server, planner_server, behavior_server,
    #      bt_navigator, waypoint_follower, velocity_smoother, lifecycle_manager
    #
    #    AMCL / map_server are NOT launched here because RTAB-Map provides
    #    both the /map topic and the map→odom transform.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )

    # ── 2. RTAB-Map SLAM ────────────────────────────────────────────────────────
    #    Subscribes to ZED2i depth + RGB + odometry.
    #    Publishes:  /map  and  map→odom TF
    #    ZED provides: odom→base_link TF via visual-inertial odometry
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
            'approx_sync': True,
            'wait_for_transform': 0.5,
            'use_sim_time': use_sim_time,
            'sync_queue_size': 30,
            'topic_queue_size': 30,
            # QoS = 1 → BEST_EFFORT to match the ZED wrapper's publisher QoS.
            'qos_image': 1,
            'qos_camera_info': 1,
            # Odometry from TF (odom->base_link), NOT from a topic.
            # This avoids timestamp sync issues between odom and image topics.
            'subscribe_odom': False,
            # RTAB-Map database location
            'database_path': '~/.ros/rtabmap.db',
            # Grid map parameters
            'Grid/FromDepth': 'true',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/RangeMax': '5.0',
            # Visual features
            'Vis/MaxFeatures': '500',
        }],
        remappings=[
            # RGB + Depth from ZED — no odom remap, using TF instead
            ('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
            ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
        ],
    )

    # ── 3. RTAB-Map Viz (publishes /rtabmap/cloud_map etc.) ─────────────────────
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('odom', '/odometry/local'),
            ('rgb/image', '/zed/zed_node/rgb/image_rect_color'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            ('depth/image', '/zed/zed_node/depth/depth_registered'),
        ],
        # Set condition to only launch if desired (can be commented out)
        # condition=IfCondition(LaunchConfiguration('rtabmap_viz', default='false')),
    )

    # ── 4. Robot Localization (Sensor Fusion) ───────────────────────────────────
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[('odometry/filtered', '/odometry/local')]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('imu', '/zed/zed_node/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/local')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_model_arg,
        ekf_local,
        navsat_transform,
        nav2,
        rtabmap,
        # Uncomment rtabmap_viz below if you want the RTAB-Map visualization window:
        # rtabmap_viz,
    ])
