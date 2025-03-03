import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', os.path.join(
        get_package_share_directory('omnidirectional_bringup'),'world','test_w_1.world')],
            output='screen'
        )
    package_path = os.path.join( get_package_share_directory('omnidirectional_description') )
    package_path_rviz = os.path.join( get_package_share_directory('omnidirectional_bringup') )

    xacro_file = os.path.join( package_path , 'urdf', 'omni_controller.xacro' )
    rviz_config_path = os.path.join( package_path_rviz , 'rviz', 'omni_control_rviz.rviz' )
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}    

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'omni_robot'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_velocity_controller'],
        output='screen'
    )
    #Recitar: In nomine patris et filii et spiritus sancti
    config_arg = DeclareLaunchArgument(name = 'rvizconfig', default_value = rviz_config_path)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[wheel_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        config_arg,
        rviz_node           
    ])