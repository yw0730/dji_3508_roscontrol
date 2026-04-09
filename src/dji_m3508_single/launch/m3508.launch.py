from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    # 通过 launch 参数切换仿真硬件与真实 CAN 硬件。
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_gazebo = LaunchConfiguration('use_gazebo')
    transport = LaunchConfiguration('transport')
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    cmd_timeout_ms = LaunchConfiguration('cmd_timeout_ms')
    strict_activate_ack = LaunchConfiguration('strict_activate_ack')
    strict_motor_state_timeout = LaunchConfiguration('strict_motor_state_timeout')

    # 动态展开 xacro，生成 robot_description。
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('dji_m3508_single'),
            'config',
            'robot.urdf.xacro'
        ]),
        ' use_fake_hardware:=',
        use_fake_hardware,
        ' use_gazebo:=',
        use_gazebo,
        ' transport:=',
        transport,
        ' serial_port:=',
        serial_port,
        ' baudrate:=',
        baudrate,
        ' cmd_timeout_ms:=',
        cmd_timeout_ms,
        ' strict_activate_ack:=',
        strict_activate_ack,
        ' strict_motor_state_timeout:=',
        strict_motor_state_timeout,
    ])

    robot_description = {'robot_description': robot_description_content}
    controllers_file = PathJoinSubstitution([
        FindPackageShare('dji_m3508_single'),
        'config',
        'controllers.yaml'
    ])
    world_file = PathJoinSubstitution([
        FindPackageShare('dji_m3508_single'),
        'worlds',
        'm3508_empty.world'
    ])

    # 启动 ros2_control 控制管理器。
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        condition=UnlessCondition(use_gazebo),
        output='screen'
    )

    # 发布 joint TF 与机器人状态。
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': use_gazebo}
        ],
        output='screen'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file}.items(),
        condition=IfCondition(use_gazebo)
    )

    spawn_in_gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'm3508_single', '-topic', 'robot_description'],
        condition=IfCondition(use_gazebo),
        output='screen'
    )

    # 先拉起状态广播器，再拉起速度控制器。
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='true: mock_components, false: real DJI CAN hardware'
        ),
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='true: launch Gazebo and spawn robot model'
        ),
        DeclareLaunchArgument(
            'transport',
            default_value='can',
            description='real hardware transport: can or uart'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='UART device path for transport=uart'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='921600',
            description='UART baudrate for transport=uart'
        ),
        DeclareLaunchArgument(
            'cmd_timeout_ms',
            default_value='100',
            description='MCU-side velocity command timeout in ms'
        ),
        DeclareLaunchArgument(
            'strict_activate_ack',
            default_value='false',
            description='If true, fail activation when SET_MODE ACK is missing'
        ),
        DeclareLaunchArgument(
            'strict_motor_state_timeout',
            default_value='false',
            description='If true, return read ERROR when MOTOR_STATE times out'
        ),
        gazebo,
        control_node,
        robot_state_publisher_node,
        spawn_in_gazebo,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
    ])
