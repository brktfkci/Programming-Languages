import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetLaunchConfiguration, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

config_dir = os.path.join(get_package_share_directory('planner2'), 'cfg')
rcfg = 'file://' + os.path.join(config_dir, 'right.ini')
lcfg = 'file://' + os.path.join(config_dir, 'left.ini')

def generate_launch_description():
    dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
        output='screen'
    )
    
    mavros_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                    FindPackageShare("planner2"), '/launch', '/mavros.py']))
    
    odometry_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                    FindPackageShare("stereo_visual_odometry"), '/launch', '/stereo_visual_odom.launch.py']))

    composable_nodes = [
        ComposableNode(
            package='gscam2',
            plugin='gscam2::GSCamNode',
            name='right_publisher',
            namespace='right',
            parameters=[{
                'gscam_config': 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1920, height=1080, format=(string)NV12 ! queue ! videoconvert',
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
                'gscam_config': 'nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1280, height=720, format=(string)NV12 ! queue ! videoconvert',
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
                ('disparity', '/disparity')
            ]
        ),
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::PointCloudNode',
            name='pointcloud_node',
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'avoid_point_cloud_padding': LaunchConfiguration('avoid_point_cloud_padding'),
                'use_color': LaunchConfiguration('use_color'),
                'use_system_default_qos': LaunchConfiguration('use_system_default_qos'),
            }],
            remappings=[
                ('left/camera_info', '/left/camera_info'),
                ('right/camera_info', '/right/camera_info'),
                ('left/image_rect_color', '/left/image_rect'),
                ('disparity', '/disparity'),
                ('points2', '/points2')
            ]
        )
    ]
    
    stereo_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stereo_link_tf',
        arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'base_link', 'stereo_link']
    )

    left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_camera_tf',
        arguments=['0.03', '0', '0', '0', '0', '0', 'stereo_link', 'left_camera']
    )

    right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_camera_tf',
        arguments=['-0.03', '0', '0', '0', '0', '0', 'stereo_link', 'right_camera']
    )

    planner = Node(
            package='planner2',
            executable='local_planner_node',
            name='local_planner',
            parameters=[
                {'goal_x': 10.0},
                {'goal_y': 10.0},
                {'goal_z': 2.0},
                {'fov_horizontal': 70.0},
                {'fov_vertical': 55.0},
                {'robot_radius': 0.5},
                {'safe_distance': 5.0},
                {'speed': 1.0}
            ]
        )

    return LaunchDescription([
        DeclareLaunchArgument('approximate_sync', default_value='True'),
        DeclareLaunchArgument('avoid_point_cloud_padding', default_value='True'),
        DeclareLaunchArgument('use_color', default_value='True'),
        DeclareLaunchArgument('queue_size', default_value='10'),
        DeclareLaunchArgument('use_system_default_qos', default_value='False'),
        DeclareLaunchArgument('stereo_algorithm', default_value='1'),
        DeclareLaunchArgument('prefilter_size', default_value='33'),
        DeclareLaunchArgument('prefilter_cap', default_value='9'),
        DeclareLaunchArgument('correlation_window_size', default_value='5'),
        DeclareLaunchArgument('min_disparity', default_value='0'),
        DeclareLaunchArgument('disparity_range', default_value='64'),
        DeclareLaunchArgument('texture_threshold', default_value='173'),
        DeclareLaunchArgument('speckle_size', default_value='0'),
        DeclareLaunchArgument('speckle_range', default_value='11'),
        DeclareLaunchArgument('disp12_max_diff', default_value='7'),
        DeclareLaunchArgument('uniqueness_ratio', default_value='15.0'),
        DeclareLaunchArgument('P1', default_value='640.0'),
        DeclareLaunchArgument('P2', default_value='840.0'),
        DeclareLaunchArgument('full_dp', default_value='False'),
        DeclareLaunchArgument('container', default_value=''),


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
        ),
        
        #dds_agent,
        #odometry_launch,
        planner,
        stereo_tf,
        left_tf,
        right_tf,
    ])