import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from xacro import process_file

def generate_launch_description():
    pkg_share = get_package_share_directory('dna_swarm_bot')
    urdf_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    robot_description_config = process_file(urdf_path, mappings={'prefix': ''})
    robot_description = robot_description_config.toxml()

    ld = LaunchDescription()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    ld.add_action(rsp)

    ld.add_action(Node(
        package='dna_swarm_bot',
        executable='arduino_bridge',
        parameters=[{'serial_port': '/dev/ttyACM0', 'wheel_separation': 0.17}],
        output='screen'
    ))

    ld.add_action(Node(
        package='dna_swarm_bot',
        executable='imu_odometry',
        output='screen'
    ))

    ld.add_action(Node(
        package='dna_swarm_bot',
        executable='marker_detector',
        parameters=[{'camera_topic': 'camera/image_raw', 'camera_info_topic': 'camera/camera_info'}],
        output='screen'
    ))

    ld.add_action(Node(
        package='dna_swarm_bot',
        executable='swarm_coordinator',
        parameters=[{'bot_id': 0, 'num_bots': 2}],
        output='screen'
    ))

    return ld
