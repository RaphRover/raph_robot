import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'
    
    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')

    name = LaunchConfiguration('name').perform(context)

    #parameters = [
    #    {
    #        "frame_id": "base_footprint",
    #        "subscribe_rgb": True,
    #        "subscribe_depth": True,
    #        "subscribe_odom_info": False,
    #        "approx_sync": True,
    #        "Rtabmap/DetectionRate": "3.5",
    #    }
    #]
    #remappings = [
    #    ("rgb/image", name+"/rgb/image_rect"),
    #    ("rgb/camera_info", name+"/rgb/camera_info"),
    #    ("depth/image", name+"/stereo/image_raw"),
    #    ("odom", "/controller/odom"),
    #]

    return [
        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file],
                        remappings=[('oak/imu/data', 'oak/imu/data_raw'),]
                    ),
                    ComposableNode(
                        package='depthimage_to_laserscan',
                        plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                        name='depthimage_to_laserscan',
                        parameters=[{'range_min': 0.3, 'range_max': 3.0, 'scan_height': 20, 'output_frame': 'oak_base_link'}],

                        remappings=[('depth', 'oak/stereo/image_raw'),
                                    ('depth_camera_info', 'oak/stereo/camera_info')]
                    ),
                    #ComposableNode(
                    #    package='imu_filter_madgwick',
                    #    plugin='ImuFilterMadgwickRos',
                    #    name='imu_madgwick_filter',
                    #    parameters=[{'use_mag': False, 'publish_tf': False}],
                    #    remappings=[('imu/data_raw', 'oak/imu/data_raw'),
                    #                ('imu/data', 'oak/imu/data')]
                    #),
                    #ComposableNode(
                    #    package="image_proc",
                    #    plugin="image_proc::RectifyNode",
                    #    name="rectify_color_node",
                    #    remappings=[('image', name+'/rgb/image_raw'),
                    #                ('camera_info', name+'/rgb/camera_info'),
                    #                ('image_rect', name+'/rgb/image_rect'),
                    #                ('image_rect/compressed', name+'/rgb/image_rect/compressed'),
                    #                ('image_rect/compressedDepth', name+'/rgb/image_rect/compressedDepth'),
                    #                ('image_rect/theora', name+'/rgb/image_rect/theora')]
                    #),
                    #ComposableNode(
                    #    package='rtabmap_slam',
                    #    plugin='rtabmap_slam::CoreWrapper',
                    #    name='rtabmap',
                    #    parameters=parameters,
                    #    remappings=remappings,
                    #),
                    #PointCloudXyziNode moved to leo2_nav/navigation.launch as a component for performance
                    
                    #ComposableNode(
                    #    package='depth_image_proc',
                    #    plugin='depth_image_proc::PointCloudXyziNode',
                    #    name='point_cloud_xyzi',
                    #    parameters=[{'queue_size': 1}],
#
                    #    remappings=[('depth/image_rect', name+'/stereo/image_raw'),
                    #                ('intensity/image_rect', name+'/right/image_raw'),
                    #                ('intensity/camera_info', name+'/right/camera_info'),
                    #                ('points', name+'/points')]
                    #),
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        ),
    ]


def generate_launch_description():
    leo2_oak_prefix = get_package_share_directory("leo2_oak")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(leo2_oak_prefix, 'config', 'pcl_oak.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )