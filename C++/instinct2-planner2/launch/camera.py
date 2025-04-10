import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetLaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

config_dir = os.path.join(get_package_share_directory('instinct2'), 'cfg')
rcfg = 'file://' + os.path.join(config_dir, 'right.ini')
lcfg = 'file://' + os.path.join(config_dir, 'left.ini')

def generate_launch_description():

    composable_nodes = [
        ComposableNode(
            package='gscam2',
            plugin='gscam2::GSCamNode',
            name='right_publisher',
            namespace='right',
            parameters=[{
                'gscam_config': 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1280, height=720, format=(string)BGRx ! queue ! videoconvert',
                'preroll': False,
                'use_gst_timestamps': False,
                'frame_id': 'right_camera',
                'camera_name': 'right',
                'camera_info_url': rcfg,
            }],
            remappings=[
                ('/image_raw', '/right/image_raw'),
                ('/camera_info', '/right/camera_info'),
            ]
        ),
        ComposableNode(
            package='gscam2',
            plugin='gscam2::GSCamNode',
            name='left_publisher',
            namespace='left',
            parameters=[{
                'gscam_config': 'nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1280, height=720, format=(string)BGRx ! queue ! videoconvert',
                'preroll': False,
                'use_gst_timestamps': False,
                'frame_id': 'left_camera',
                'camera_name': 'left',
                'camera_info_url': lcfg,
            }],
            remappings=[
                ('/image_raw', '/left/image_raw'),
                ('/camera_info', '/left/camera_info'),
            ]
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_right',
            namespace='right',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
                ('camera_info_rect', 'camera_info_rect')
            ]
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_left',
            namespace='left',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
                ('camera_info_rect', 'camera_info_rect')
            ]
        ),
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            name='disparity_node',
            namespace='',
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
                'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                'prefilter_size': LaunchConfiguration('prefilter_size'),
                'prefilter_cap': LaunchConfiguration('prefilter_cap'),
                'correlation_window_size': LaunchConfiguration('correlation_window_size'),
                'min_disparity': LaunchConfiguration('min_disparity'),
                'disparity_range': LaunchConfiguration('disparity_range'),
                'texture_threshold': LaunchConfiguration('texture_threshold'),
                'speckle_size': LaunchConfiguration('speckle_size'),
                'speckle_range': LaunchConfiguration('speckle_range'),
                'disp12_max_diff': LaunchConfiguration('disp12_max_diff'),
                'uniqueness_ratio': LaunchConfiguration('uniqueness_ratio'),
                'P1': LaunchConfiguration('P1'),
                'P2': LaunchConfiguration('P2'),
                'full_dp': LaunchConfiguration('full_dp'),
                'queue_size': LaunchConfiguration('queue_size'),
            }],
            remappings=[
                ('left/image_rect', '/left/image_rect'),
                ('left/camera_info', '/left/camera_info'),
                ('right/image_rect', '/right/image_rect'),
                ('right/camera_info', '/right/camera_info'),
                ('disparity', '/stereo/disparity')
            ]
        )
    ]

    return LaunchDescription([
        DeclareLaunchArgument('approximate_sync', default_value='True'),            # Синхронизация топиков. Установить если левая и правая камеры не выдают синхронизированные timestamp
        DeclareLaunchArgument('avoid_point_cloud_padding', default_value='False'),  # Избегать выравнивания заполнения в сгенерированном облаке точек.
        DeclareLaunchArgument('use_color', default_value='True'),                   # Генерировать облако точек с данными RGB.
        DeclareLaunchArgument('use_system_default_qos', default_value='False'),     # Использовать настройки QoS RMW по умолчанию для подписки на изображения и информацию о камере.
        DeclareLaunchArgument('launch_image_proc', default_value='True'),           # Запускать ли узлы дебайеризации и ректификации из image_proc.
        DeclareLaunchArgument('left_namespace', default_value='left'),              # Пространство имен для левой камеры.
        DeclareLaunchArgument('right_namespace', default_value='right'),            # Пространство имен для правой камеры.

        DeclareLaunchArgument('stereo_algorithm', default_value='1'),               # Стерео алгоритм: Блочное соответствие (0) или Полуглобальное блочное соответствие (1).
        DeclareLaunchArgument('prefilter_size', default_value='19'),                # Размер окна нормализации в пикселях (должен быть нечетным).
        DeclareLaunchArgument('prefilter_cap', default_value='13'),                 # Ограничение на нормализованные значения пикселей.
        DeclareLaunchArgument('correlation_window_size', default_value='5'),        # Ширина окна корреляции SAD в пикселях (должна быть нечетной).
        DeclareLaunchArgument('min_disparity', default_value='0'),                  # испаратность, с которой начинается поиск, в пикселях.
        DeclareLaunchArgument('disparity_range', default_value='176'),              # Количество диспаратностей для поиска в пикселях (должно быть кратно 16).
        DeclareLaunchArgument('texture_threshold', default_value='255'),            # Отфильтровывать, если отклик окна SAD не превышает порог текстуры.
        DeclareLaunchArgument('speckle_size', default_value='13'),                  # Отклонять регионы меньше этого размера в пикселях.
        DeclareLaunchArgument('speckle_range', default_value='19'),                 # Максимально допустимая разница между обнаруженными диспаратностями.
        DeclareLaunchArgument('disp12_max_diff', default_value='7'),                # (SGBM)Максимально допустимая разница в проверке диспаратности лево-право в пикселях
        DeclareLaunchArgument('uniqueness_ratio', default_value='8.0'),             # Отфильтровывать, если лучшее соответствие недостаточно превышает следующее лучшее соответствие.
        DeclareLaunchArgument('P1', default_value='640.0'),                         # (SGBM)Первый параметр, контролирующий гладкость диспаратности 
        DeclareLaunchArgument('P2', default_value='840.0'),                         # (SGBM)Второй параметр, контролирующий гладкость диспаратности
        DeclareLaunchArgument('container', default_value=''),                       # Имя существующего контейнера узлов, в который будут загружены запускаемые узлы. При пустом значении создает новый контейнер.
        DeclareLaunchArgument('full_dp', default_value='False'),                    # (SGBM)Использовать полноценную версию алгоритма
        DeclareLaunchArgument('queue_size', default_value='10'),                    # Размер очереди сообщений
        

        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            package='rclcpp_components',
            executable='component_container',
            name='stereo_image_proc_container',
            namespace='',
            composable_node_descriptions=composable_nodes,
            output='screen'
        ),
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', ''),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration('container'),
        ),
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals('container', ''),
            name='container',
            value=PythonExpression([
                '"stereo_image_proc_container"', ' if ',
                '"', LaunchConfiguration('ros_namespace', default=''), '"',
                ' == "" else ', '"',
                LaunchConfiguration('ros_namespace', default=''), '/stereo_image_proc_container"'
            ]),
        )
    ])