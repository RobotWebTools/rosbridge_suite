from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # defaults
    port="9090"
    address=""
    ssl="false"
    certfile=""
    keyfile=""
    retry_startup_delay="5.0"
    fragment_timeout="600"
    delay_between_messages="0"
    max_message_size="10000000"
    unregister_timeout="10.0"
    use_compression="false"
    topics_glob=""
    services_glob=""
    params_glob=""
    bson_only_mode=""

    # Package Directories
    pkg_rosbridge_server = get_package_share_directory("rosbridge_server")

    # TODO add ssl version and condition...
    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        arguments=[],
        parameters=[{
          "port": port,
          "address": address,
          "retry_startup_delay": retry_startup_delay,
          "fragment_timeout": fragment_timeout,
          "delay_between_messages": delay_between_messages,
          "max_message_size": max_message_size,
          "unregister_timeout": unregister_timeout,
          "use_compression": use_compression,
          "topics_glob": topics_glob,
          "services_glob": services_glob,
          "params_glob": params_glob,
          "bson_only_mode": bson_only_mode,
        }],
        remappings=[],
        output="screen",
    )

    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        arguments=[],
        parameters=[{
            "topics_glob": topics_glob,
            "services_glob": services_glob,
            "params_glob": params_glob,
        }],
        remappings=[],
        output="screen",
    )

    # Nodes and Launches
    return LaunchDescription(
        [
          rosbridge_websocket,
          rosapi,
        ]
    )
