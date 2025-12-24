import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

PACKAGE_NAME = "my_omni_robot"


def generate_launch_description():
    pkg_path = get_package_share_directory(PACKAGE_NAME)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # SRDF
    srdf_file = os.path.join(pkg_path, 'config', 'moveit', 'my_omni_robot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Kinematics
    kinematics_yaml = os.path.join(pkg_path, 'config', 'moveit', 'kinematics.yaml')
    
    # Joint limits
    joint_limits_yaml = os.path.join(pkg_path, 'config', 'moveit', 'joint_limits.yaml')

    # OMPL planning
    ompl_planning_yaml = os.path.join(pkg_path, 'config', 'moveit', 'ompl_planning.yaml')

    # MoveIt controllers
    moveit_controllers_yaml = os.path.join(pkg_path, 'config', 'moveit', 'moveit_controllers.yaml')

    # Move Group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
            kinematics_yaml,
            ompl_planning_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
        ]
    )

    # RViz with MoveIt plugin
    rviz_config = os.path.join(pkg_path, 'rviz', 'moveit_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
            kinematics_yaml,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        move_group_node,
        rviz_node
    ])
