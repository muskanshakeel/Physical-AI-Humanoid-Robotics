import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the path to the URDF file
    # For this example, we assume it's in the same directory as this launch file
    # In a real ROS 2 package, it would be in a 'share' directory
    urdf_file_path = os.path.join(
        os.getcwd(), # This is usually get_package_share_directory('your_package_name'),
        'simple_robot.urdf'
    )

    # Robot State Publisher Node
    # Publishes the robot model to tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file_path, 'r').read()}],
        arguments=[urdf_file_path] # This might be redundant depending on ROS 2 version
    )

    # Gazebo Launch
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn the URDF model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity
    ])
