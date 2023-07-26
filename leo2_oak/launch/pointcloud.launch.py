import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    leo2_oak_prefix = get_package_share_directory("leo2_oak")

    name = LaunchConfiguration('name').perform(context)
    
    return [

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(leo2_oak_prefix, 'launch', 'oak_camera.launch.py')),
            launch_arguments={"name": name,
                              "params_file": params_file,
                               }.items()),

        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                    ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyziNode',
                    name='point_cloud_xyzi',
                    parameters=[{'queue_size': 1}],

                    remappings=[('depth/image_rect', name+'/stereo/image_raw'),
                                ('intensity/image_rect', name+'/right/image_raw'),
                                ('intensity/camera_info', name+'/right/camera_info'),
                                ('points', name+'/points')
                                ]),
            ],
        ),
    ]


def generate_launch_description():
    leo2_oak_prefix = get_package_share_directory("leo2_oak")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(leo2_oak_prefix, 'config', 'pcl.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
