
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
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

    # Argumento para definir o namespace do drone
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='uav1',  # Valor padrão: 'uav1'
            description='Top-level namespace for the UAV.'
        )
    )

    # Argumento para o arquivo de parâmetros do imu_filter
    declared_arguments.append(
        DeclareLaunchArgument(
            'imu_filter_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_vins_imu_filter'),
                                                'params', 'imu_filter.yaml']),
            description='Full path to the file with the waypoint loader parameters.'
        )
    )

    # Inicialização dos argumentos
    namespace = LaunchConfiguration('namespace')
    imu_filter_file = LaunchConfiguration('imu_filter_file')

    # Definição do LifecycleNode
    waypoint_lifecycle_node = LifecycleNode(
        package='laser_vins_imu_filter',  # Nome do pacote
        executable='vins_imu_filter',  # Nome do executável
        name='vins_imu_filter',  # Nome do nó
        namespace=namespace,  # Namespace do nó
        output='screen',  # Saída do log no terminal
        parameters=[imu_filter_file],  # Carrega os parâmetros do arquivo YAML
        remappings=[
            # Remapeamentos de tópicos
            ('imu_in', '/uav1/rgbd/imu'),  # Remapeia tópico de entrada
            ('imu_out', '/uav1/rgbd/imu_filtered'),  # Remapeia tópico de saída
        ]
    )

    # Lista de manipuladores de eventos
    event_handlers = []

    # Manipulador para realizar a transição para o estado 'configure' assim que o nó for iniciado
    event_handlers.append(
        RegisterEventHandler(
            OnProcessStart(
                target_action=waypoint_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(waypoint_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    # Manipulador para realizar a transição para o estado 'activate' após o estado 'configuring'
    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=waypoint_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(waypoint_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    # Criação do LaunchDescription
    ld = LaunchDescription()

    # Declaração dos argumentos
    for argument in declared_arguments:
        ld.add_action(argument)

    # Adiciona o nó do imu_filter
    ld.add_action(waypoint_lifecycle_node)

    # Adiciona os manipuladores de eventos
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
