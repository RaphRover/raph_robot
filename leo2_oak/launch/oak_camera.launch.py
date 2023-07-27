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
                    ),
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