from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    # === Declaração de argumentos ===
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value=EnvironmentVariable('UAV_NAME'),
            description='Top-level namespace for the UAV.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'imu_filter_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('laser_vins_imu_filter'),
                'params', 'imu_filter.yaml']),
            description='Full path to the file with the imu_filter parameters.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_accel',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/front_rgbd/accel/sample'],
            description='Name of the accel topic.')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_gyro',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/front_rgbd/gyro/sample'],
            description='Name of the gyro topic.')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_imu_in',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/vio_sensor/imu/data'],
            description='Name of the raw IMU input topic.')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_imu_out',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/imu_filtered/imu'],
            description='Name of the filtered IMU output topic.')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_accel_gyro_out',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/imu_filtered/accel_gyro'],
            description='Name of the filtered IMU output topic.')
    )

    # === Recupera argumentos com LaunchConfiguration ===
    namespace = LaunchConfiguration('namespace')
    imu_filter_file = LaunchConfiguration('imu_filter_file')
    topic_accel = LaunchConfiguration('topic_accel')
    topic_gyro = LaunchConfiguration('topic_gyro')
    topic_imu_in = LaunchConfiguration('topic_imu_in')
    topic_imu_out = LaunchConfiguration('topic_imu_out')
    topic_accel_gyro_out = LaunchConfiguration('topic_accel_gyro_out')

    # === Nó Lifecycle ===
    vins_imu_filter_node = LifecycleNode(
        package='laser_vins_imu_filter',
        executable='vins_imu_filter',
        name='vins_imu_filter',
        namespace=namespace,
        output='screen',
        parameters=[imu_filter_file],
        remappings=[
            ('accel_in', topic_accel),
            ('gyro_in', topic_gyro),
            ('imu_in', topic_imu_in),
            ('imu_out', topic_imu_out),
            ('accel_gyro_out', topic_accel_gyro_out),
        ]
    )

    # === Eventos para configurar e ativar automaticamente ===
    event_handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=vins_imu_filter_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(vins_imu_filter_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ]
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=vins_imu_filter_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(vins_imu_filter_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        )
    ]

    # === Descrição do lançamento ===
    ld = LaunchDescription()

    for arg in declared_arguments:
        ld.add_action(arg)

    ld.add_action(vins_imu_filter_node)

    for handler in event_handlers:
        ld.add_action(handler)

    return ld
