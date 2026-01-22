import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value=os.getenv('UAV_NAME', "uav1"),
            description='Top-level namespace.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=PythonExpression(['"', os.getenv('REAL_UAV', "true"), '" == "false"']),
            description='Whether use the simulation time.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'vins_imu_filter_params_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_vins_imu_filter'),
                                                'params', 'px4_api.yaml']),
            description='Full path to the file with the parameters.'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'imu_data_united',
            default_value='True',
            description='Is the IMU data united (one topic) or separated (accel and gyro topics)?'))

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_accel_in',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/px4_api/accel/sample'],
            description='Name of the accel topic.')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_gyro_in',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/px4_api/gyro/sample'],
            description='Name of the gyro topic.')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_imu_in',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/px4_api/imu_BAG'],
            description='Name of the raw IMU input topic.')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'topic_imu_out',
            default_value=['/', EnvironmentVariable('UAV_NAME'), '/px4_api/filtered/imu'],
            description='Name of the filtered IMU output topic.'))

    # Initialize arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vins_imu_filter_params_file = LaunchConfiguration('vins_imu_filter_params_file')
    imu_data_united = LaunchConfiguration('imu_data_united')
    topic_accel_in = LaunchConfiguration('topic_accel_in')
    topic_gyro_in = LaunchConfiguration('topic_gyro_in')
    topic_imu_in = LaunchConfiguration('topic_imu_in')
    topic_imu_out = LaunchConfiguration('topic_imu_out')

    # Declare nodes
    vins_imu_filter_lifecycle_node = LifecycleNode(
        package='laser_vins_imu_filter',
        executable='vins_imu_filter',
        name='px4_imu_filter',
        namespace=namespace,
        output='screen',
        parameters=[vins_imu_filter_params_file,
                    {'imu_data_united': imu_data_united,
                     'use_sim_time': use_sim_time}],
        remappings=[('accel_in', topic_accel_in),
                    ('gyro_in', topic_gyro_in),
                    ('imu_in', topic_imu_in),
                    ('imu_out', topic_imu_out)])

    change_to_configure_state_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=vins_imu_filter_lifecycle_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(vins_imu_filter_lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))]))

    change_to_activate_state_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=vins_imu_filter_lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(vins_imu_filter_lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))

    return LaunchDescription(declared_arguments + [vins_imu_filter_lifecycle_node,
                                                   change_to_configure_state_event_handler,
                                                   change_to_activate_state_event_handler])
