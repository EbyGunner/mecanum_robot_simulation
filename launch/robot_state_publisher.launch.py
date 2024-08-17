import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'robot_package'
    urdf_file_subpath = 'description/urdf'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), urdf_file_subpath, 'urdf_final.urdf')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Declare the world file argument
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(get_package_share_directory(pkg_name), 'world', 'mecanum_world.world')],
        description='Full path to the world file to load'
    )

    # Configure the node for robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                    'use_sim_time': True}] # add other parameters here if required
    )

    # Include Gazebo launch file with the world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # Node to spawn the robot entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'Mecanum_robots',
                                '-x', '0',  # Set x position
                                '-y', '0',  # Set y position
                                '-z', '1'],  # Set z position (e.g., 1 meter above ground)],
                    output='screen')


    # Run the node
    return LaunchDescription([
        declare_world_cmd,
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])