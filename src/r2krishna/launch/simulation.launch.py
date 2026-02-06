import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_r2krishna = get_package_share_directory('r2krishna')
    pkg_arena_viz = get_package_share_directory('arena_viz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Check Packages
    slam_installed = True
    nav2_installed = True

    robot_urdf = os.path.join(pkg_r2krishna, 'urdf', 'r2krishna.urdf')
    arena_world = os.path.join(pkg_arena_viz, 'worlds', 'arena.world')
    rviz_config_file = os.path.join(pkg_r2krishna, 'config', 'view_robot.rviz')

    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    # Resource Paths
    r2_parent = os.path.abspath(os.path.join(pkg_r2krishna, '..'))
    arena_parent = os.path.abspath(os.path.join(pkg_arena_viz, '..'))
    resource_env = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{r2_parent}:{arena_parent}"
    )

    launch_nodes = []

    # 1. BRIDGE (Must be first)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/r2krishna/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/r2krishna/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{
            'qos_overrides./scan.publisher.reliability': 'reliable',
            'qos_overrides./camera/image_raw.publisher.reliability': 'reliable',
            'qos_overrides./camera/camera_info.publisher.reliability': 'reliable',
            'use_sim_time': True
        }],
        remappings=[
            ('/model/r2krishna/odometry', '/odom'),
            ('/model/r2krishna/tf', '/tf')
        ],
        output='screen'
    )
    launch_nodes.append(bridge)

    # 2. WORLD TRANSFORM (THE ANCHOR)
    # This connects 'world' to 'map' with a (2,2) offset.
    # It fixes the "Unconnected Tree" error and makes the grid appear.
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '2.0', '--y', '2.0', '--z', '0.0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'map'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    launch_nodes.append(world_tf)

    # 3. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {arena_world}'}.items(),
    )
    launch_nodes.append(gazebo)

    # 4. Spawn Robot (AT 2,2)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'r2krishna',
            '-x', '2.0', '-y', '2.0', '-z', '0.5'
        ],
        output='screen'
    )
    launch_nodes.append(spawn_robot)

    # 5. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )
    launch_nodes.append(rsp)

    # 6. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    launch_nodes.append(rviz)

    # 7. SLAM & Nav2
    if slam_installed:
        pkg_slam = get_package_share_directory('slam_toolbox')
        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items()
        )
        launch_nodes.append(slam)

    if nav2_installed:
        pkg_nav2 = get_package_share_directory('nav2_bringup')
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file': os.path.join(pkg_nav2, 'params', 'nav2_params.yaml')
            }.items()
        )
        launch_nodes.append(nav2)

    # --- SENSORS ---
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.25', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'r2krishna/base_footprint/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    launch_nodes.append(lidar_tf)

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.3', '--y', '0', '--z', '0.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'r2krishna/base_footprint/camera'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    launch_nodes.append(camera_tf)

    return LaunchDescription([resource_env] + launch_nodes)
