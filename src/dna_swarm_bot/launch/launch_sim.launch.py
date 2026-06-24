import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from xacro import process_file

def generate_launch_description():
    pkg_share = get_package_share_directory('dna_swarm_bot')
    world_path = os.path.join(pkg_share, 'worlds', 'swarm_world.sdf')
    urdf_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    num_bots = 2
    ld = LaunchDescription()

    # Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items()
    )
    ld.add_action(gazebo)

    # Spawn wall markers using gz service
    wall_positions = [3.0, 6.0, 9.0]
    wall_ids = [10, 11, 12]
    for wx, wid in zip(wall_positions, wall_ids):
        spawn_wall = ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/swarm_world/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000',
                 '--req', f'sdf_filename: "{pkg_share}/models/wall_marker_{wid}/model.sdf" name: "wall_marker_{wid}" pose: {{position: {{x: {wx}, y: 0.0, z: 0.1}}}}'],
            output='screen'
        )
        ld.add_action(spawn_wall)

    for i in range(num_bots):
        prefix = f'bot_{i}'
        namespace = prefix
        marker_id = i

        robot_description_config = process_file(
            urdf_path,
            mappings={'prefix': prefix, 'bot_marker_id': str(marker_id)}
        )
        robot_description = robot_description_config.toxml()

        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True,
                         'frame_prefix': f'{namespace}/'}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
        ld.add_action(rsp)

        # Spawn robot using gz service
        spawn_bot = ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/swarm_world/create',
                 '--reqtype', 'gz.msgs.EntityFactory',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000',
                 '--req', f'sdf_string: "{robot_description.replace(chr(10), " ").replace(chr(34), chr(92)+chr(34))}" name: "dna_bot_{i}" pose: {{position: {{x: {i*0.35}, y: 0.0, z: 0.1}}}}'],
            output='screen'
        )
        ld.add_action(spawn_bot)

        # Real marker detector processing actual camera feed
        marker_det = Node(
            package='dna_swarm_bot',
            executable='marker_detector',
            namespace=namespace,
            parameters=[{
                'marker_size': 0.05,
                'camera_topic': 'camera/image_raw',
                'camera_info_topic': 'camera/camera_info',
                'bot_marker_ids': [0, 1],
                'wall_marker_ids': [10, 11, 12],
                'use_sim_time': True
            }],
            output='screen'
        )
        ld.add_action(marker_det)

        coord = Node(
            package='dna_swarm_bot',
            executable='swarm_coordinator',
            namespace=namespace,
            parameters=[{'bot_id': i, 'num_bots': num_bots, 'use_sim_time': True}],
            output='screen'
        )
        ld.add_action(coord)

    return ld
