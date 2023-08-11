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
                    #Rtabmap odom nodes
                    """
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name="right_rectify_mono_node",
                        remappings=[('image', '/oak/right/image_raw'),
                                    ('camera_info', '/oak/right/camera_info'),
                                    ('image_rect', '/oak/right/image_rect'),
                                    ('image_rect/compressed', '/oak/right/image_rect/compressed'),
                                    ('image_rect/compressedDepth', '/oak/right/image_rect/compressedDepth'),
                                    ('image_rect/theora', '/oak/right/image_rect/theora')]
                    ),
                    ComposableNode(
                        package="image_proc",
                        plugin="image_proc::RectifyNode",
                        name="left_rectify_mono_node",
                        remappings=[('image', '/oak/left/image_raw'),
                                    ('camera_info', '/oak/left/camera_info'),
                                    ('image_rect', '/oak/left/image_rect'),
                                    ('image_rect/compressed', '/oak/left/image_rect/compressed'),
                                    ('image_rect/compressedDepth', '/oak/left/image_rect/compressedDepth'),
                                    ('image_rect/theora', '/oak/left/image_rect/theora')]
                    ),
                    ComposableNode(
                        package='rtabmap_odom',
                        plugin='rtabmap_odom::StereoOdometry',
                        name='stereo_odometry',
                        parameters=[{
                                        "frame_id": "base_footprint",
                                        "subscribe_rgb": True,
                                        "subscribe_depth": True,
                                        "subscribe_odom_info": False,
                                        "approx_sync": True,
                                        "Rtabmap/DetectionRate": "3.5",
                                        "publish_tf": False,
                                        "wait_imu_to_init": False,
                                    }],
                        remappings=[
                                        ('right/camera_info', '/oak/right/camera_info'),
                                        ('right/image_rect', '/oak/right/image_rect'),
                                        ('left/camera_info', '/oak/left/camera_info'),
                                        ('left/image_rect', '/oak/left/image_rect'),
                                        ('imu', '/controller/imu'),
                                    ],
                    ),
                    """
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