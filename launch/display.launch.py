from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='proto1',
        description='urdf/<robot>/<robot>.xacro を表示する（proto1, gamma）'
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='joint_state_publisher_gui を起動する（スライダで関節を動かす）'
    )

    xacro_path = PathJoinSubstitution([
        FindPackageShare('rabbits_description'), 'urdf', LaunchConfiguration('robot'),
        [LaunchConfiguration('robot'), '.xacro'],
    ])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_path]), value_type=str)

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_pub_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('rabbits_description'), 'rviz', 'display.rviz'])],
    )

    return LaunchDescription([
        robot_arg,
        use_gui_arg,
        robot_state_pub_node,
        joint_state_pub_gui_node,
        rviz_node,
    ])
