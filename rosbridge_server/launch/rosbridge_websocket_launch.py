from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # defaults
    port=9090
    address=""
    ssl=False
    certfile=""
    keyfile=""
    retry_startup_delay=5.0
    fragment_timeout=600
    delay_between_messages=0
    max_message_size=10000000
    unregister_timeout=10.0
    use_compression=False
    topics_glob=""
    services_glob=""
    params_glob=""
    bson_only_mode=False

    # parameters for the rosbridge_websocket node
    if ssl:
        rosbridge_websocket_params = [{
            "certfile": certfile,
            "keyfile": keyfile,
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
        }]
    else:
        rosbridge_websocket_params = [{
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
        }]

    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        arguments=[],
        parameters=rosbridge_websocket_params,
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

    # nodes and launches
    return LaunchDescription(
        [
          rosbridge_websocket,
          rosapi,
        ]
    )
