import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "my_omni_robot"


def generate_launch_description():
    pkg_path = get_package_share_directory(PACKAGE_NAME)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'moveit_config.rviz')
    bridge_config = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.sdf')

    # SRDF
    srdf_file = os.path.join(pkg_path, 'config', 'moveit', 'my_omni_robot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # MoveIt configs
    kinematics_yaml = os.path.join(pkg_path, 'config', 'moveit', 'kinematics.yaml')
    ompl_planning_yaml = os.path.join(pkg_path, 'config', 'moveit', 'ompl_planning.yaml')
    joint_limits_yaml = os.path.join(pkg_path, 'config', 'moveit', 'joint_limits.yaml')
    moveit_controllers_yaml = os.path.join(pkg_path, 'config', 'moveit', 'moveit_controllers.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(pkg_path).parent.resolve())]
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{world_file} -r -v 4'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_omni_robot',
            '-z', '0.05'
        ],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    # Spawn all controllers
    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'wheel1_controller',
            'wheel2_controller',
            'wheel3_controller',
            'wheel4_controller',
            'arm_controller',
            'gripper_controller'
        ],
        output='screen'
    )

    # Kinematics node for mobile base
    kinematics = Node(
        package=PACKAGE_NAME,
        executable='kinematics',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # MoveIt Move Group (delayed)
    move_group = TimerAction(
        period=5.0,
        actions=[
            Node(
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
        ]
    )

    # RViz with MoveIt
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'use_sim_time': use_sim_time},
            kinematics_yaml,
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_resource_path,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        spawn_controllers,
        kinematics,
        move_group,
        rviz
    ])
