# joint_state_publisher_launch.py
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_first_pkg').find('my_first_pkg')
    default_model_path = os.path.join(pkg_share, 'urdf/test.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')    

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    #joint_state_publisher_node = launch_ros.actions.Node(
    #    package='joint_state_publisher', 
    #    executable='joint_state_publisher',
    #    name='joint_state_publisher', 
    #)
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
            
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),        
        Node(
            package='my_first_pkg',
            executable='test.py',
            name='goal_test',
            output='screen',
        ),
        
    #    joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
