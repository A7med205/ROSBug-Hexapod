import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,  
    SetLaunchConfiguration 
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    pkg_hexapod_share = get_package_share_directory('hexapod_sim')
    pkg_file_path = f"file://{pkg_hexapod_share}/"
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config_file = os.path.join(pkg_hexapod_share, 'rviz', 'config.rviz')
    robot_controllers = PathJoinSubstitution(
        [
            pkg_hexapod_share,
            'config',
            'robot_controller.yaml',
        ]
    )

    # Spawn Coordinates
    spawn_x = '0.0'
    spawn_y = '0.0'
    spawn_z = '0.25'

    # Getting the raw xacro output
    initial_robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [pkg_hexapod_share, 'urdf', 'hexapod.urdf.xacro']
            ),
        ]
    )

    # URI string modification for gz compatibility using OpaqueFunction
    def modify_robot_description(context):
        xacro_output_str = initial_robot_description.perform(context)

        modified_description_str = xacro_output_str.replace(
            "package://hexapod_sim/",
            pkg_file_path
        )
        
        set_config_action = SetLaunchConfiguration(
            name='final_robot_description',
            value=modified_description_str
        )
        return [set_config_action]

    robot_description_param = {
        'robot_description': LaunchConfiguration('final_robot_description')
    }

    # Action to execute the function
    prepare_robot_description = OpaqueFunction(
        function=modify_robot_description
    )

    # State publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    # Joint states for RViz-only visualization when Gazebo is disabled.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_sim),
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Robot spawner
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'hexapod',
                   '-allow_renaming', 'true',
                   '-x', spawn_x,
                   '-y', spawn_y,
                   '-z', spawn_z,
                  ],
        condition=IfCondition(use_sim),
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    
    # Controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--param-file',
            robot_controllers,
            '--controller-manager', '/controller_manager'
        ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(use_sim),
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])],
        condition=IfCondition(use_sim),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 if true'),
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Launch Gazebo Sim and simulated controllers if true'),

        gz_sim,
        prepare_robot_description,
        node_robot_state_publisher,
        joint_state_publisher,
        gz_spawn_entity,        

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            ),
            condition=IfCondition(use_sim),
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            ),
            condition=IfCondition(use_sim),
        ),

        bridge,
        rviz_node,
    ])
