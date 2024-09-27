import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('mecanum_robot_simulation')
    bridge_params = os.path.join(package_path, 'config', 'mecanum_robot_waffle_bridge.yaml')

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(package_path, 'worlds'), ':' +
            str(Path(package_path).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value=os.path.join(package_path, 'world', 'mecanum_world'),
                description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r']
                    )
                ]
             )

    xacro_file = os.path.join(package_path, 'description', 'urdf', 'urdf_final.urdf')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '1.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'mecanum_robot',
                   '-allow_renaming', 'false'],
    )

    # Bridge
    bridge = Node(package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[{'config_file': bridge_params},],
            output='screen',
            )
    
    start_gazebo_ros_image_bridge_cmd = Node(package='ros_gz_image',
                                            executable='image_bridge',
                                            arguments=['/camera'],
                                            output='screen',
                                        )

    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[load_joint_state_controller],
        #     )
        # ),
        gazebo_resource_path,
        arguments,
        gazebo,
        # node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        start_gazebo_ros_image_bridge_cmd
    ])
