from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumentos principales
    agents_arg = DeclareLaunchArgument(
        'agentes',
        default_value='all',
        description='Lista de IDs de robots a lanzar (ej: "3,5,8" o "all" o "5" para 1..5)'
    )

    meta2_arg = DeclareLaunchArgument(
        'meta2',
        default_value='',
        description='IDs de robots que usarán /Meta_2/pose'
    )
    meta3_arg = DeclareLaunchArgument(
        'meta3',
        default_value='',
        description='IDs de robots que usarán /Meta_3/pose'
    )
    meta4_arg = DeclareLaunchArgument(
        'meta4',
        default_value='',
        description='IDs de robots que usarán /Meta_4/pose'
    )

    # Configuración de los robots
    robots = {
        3: 270.1823,
        4: 278.0994,
        5: 271.1894,
        6: 270.0347,
        7: 265.9134,
        8: 270.0128,
        9: 265.7413,
        11: 263.0,
        12: 263.0,
        13: 261.0,
    }

    def parse_ids(raw_value: str):
        """Convierte una cadena tipo '3,5,8' en una lista [3,5,8]."""
        if not raw_value:
            return []
        try:
            return [int(x.strip()) for x in raw_value.split(',') if x.strip()]
        except Exception:
            return []

    def launch_selected_nodes(context, *args, **kwargs):
        # Leer argumentos
        val = context.launch_configurations.get('agentes', 'all')
        meta2_str = context.launch_configurations.get('meta2', '')
        meta3_str = context.launch_configurations.get('meta3', '')
        meta4_str = context.launch_configurations.get('meta4', '')

        meta2_ids = parse_ids(meta2_str)
        meta3_ids = parse_ids(meta3_str)
        meta4_ids = parse_ids(meta4_str)

        # Determinar qué robots lanzar
        if val == 'all':
            selected_nodes = list(robots.keys())
        elif val.isdigit():
            n = int(val)
            selected_nodes = [i for i in robots if i <= n]
        else:
            try:
                selected_nodes = [int(x.strip()) for x in val.split(',')]
            except Exception:
                selected_nodes = []

        actions = []
        for robot_id in selected_nodes:
            offset = robots[robot_id]

            # Determinar meta según el ID
            if robot_id in meta2_ids:
                goal_topic = '/Meta_2/pose'
            elif robot_id in meta3_ids:
                goal_topic = '/Meta_3/pose'
            elif robot_id in meta4_ids:
                goal_topic = '/Meta_4/pose'
            else:
                goal_topic = '/Meta_1/pose'

            actions.append(LogInfo(msg=f'✅ Pololu {robot_id} → {goal_topic}'))

            actions.append(
                Node(
                    package='pololu_controller',
                    executable='cp2p_pololu',
                    name=f'P{robot_id}controller',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'pose_topic': f'/pololu_{robot_id}/pose'},
                        {'goal_topic': goal_topic},
                        {'cmd_vel_topic': f'/cmd_vel_{robot_id}'},
                        {'offset_deg': offset},
                    ],
                )
            )
        return actions

    # Devolver descripción del lanzamiento
    return LaunchDescription([
        agents_arg,
        meta2_arg,
        meta3_arg,
        meta4_arg,
        OpaqueFunction(function=launch_selected_nodes),
    ])

