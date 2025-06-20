import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_first_pkg').find('my_first_pkg')
    default_model_path = os.path.join(pkg_share, 'urdf/my_robot.urdf')    
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/simulate.rviz')

    simulate_node = launch_ros.actions.Node(
        package='my_first_pkg',
        executable='simulate_tf',
        name='simulate_tf',
        output='screen',
    )
    diff_tf_node = launch_ros.actions.Node(
        package='my_first_pkg',
        executable='diff_tf.py',
        name='diff_tf',
        output='screen',
    )    

    teleop_twist_key_node = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix = 'xterm -e', #새로운 terminal 창으로 띄우는 gui 창
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    #joint_state_publisher_gui_node = launch_ros.actions.Node(
    #    package='joint_state_publisher_gui', #joint_state_publisher_gui 를 통해 gui 창으로 조종가능 
    #    executable='joint_state_publisher_gui', #joint_state_publisher_gui 를 통해 gui 창으로 조종가능
    #    name='joint_state_publisher_gui', #joint_state_publisher_gui 를 통해 gui 창으로 조종가능
    #)
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen', #터미널 출력 여부
        arguments=['-d', LaunchConfiguration('simulateconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
                
        launch.actions.DeclareLaunchArgument(name='simulateconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        simulate_node,
        diff_tf_node,
        teleop_twist_key_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])