#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from std_msgs.msg import Int32
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.netutil import bind_sockets
from tornado.web import Application

from rosbridge_server import ClientManager, RosbridgeWebSocket


def start_hook():
    IOLoop.instance().start()


def shutdown_hook():
    IOLoop.instance().stop()


class RosbridgeWebsocketNode(Node):
    def __init__(self):
        super().__init__("rosbridge_websocket")

        RosbridgeWebSocket.node_handle = self

        ##################################################
        # Parameter handling                             #
        ##################################################

        self.protocol_parameter_handling()

        # get tornado application parameters
        tornado_settings = {}
        tornado_settings["websocket_ping_interval"] = float(
            self.declare_parameter("websocket_ping_interval", 0).value
        )
        tornado_settings["websocket_ping_timeout"] = float(
            self.declare_parameter("websocket_ping_timeout", 30).value
        )
        if "--websocket_ping_interval" in sys.argv:
            idx = sys.argv.index("--websocket_ping_interval") + 1
            if idx < len(sys.argv):
                tornado_settings["websocket_ping_interval"] = float(sys.argv[idx])
            else:
                print("--websocket_ping_interval argument provided without a value.")
                sys.exit(-1)

        if "--websocket_ping_timeout" in sys.argv:
            idx = sys.argv.index("--websocket_ping_timeout") + 1
            if idx < len(sys.argv):
                tornado_settings["websocket_ping_timeout"] = float(sys.argv[idx])
            else:
                print("--websocket_ping_timeout argument provided without a value.")
                sys.exit(-1)

        # Server and SSL options
        # SSL options, cannot set default to None - rclpy throws warning
        certfile = self.declare_parameter("certfile", "").value
        keyfile = self.declare_parameter("keyfile", "").value
        # if not set, set to None
        if certfile == "":
            certfile = None
        if keyfile == "":
            keyfile = None

        port = self.declare_parameter("port", 9090).value
        if "--port" in sys.argv:
            idx = sys.argv.index("--port") + 1
            if idx < len(sys.argv):
                port = int(sys.argv[idx])
            else:
                print("--port argument provided without a value.")
                sys.exit(-1)
        address = self.declare_parameter("address", "").value
        if "--address" in sys.argv:
            idx = sys.argv.index("--address") + 1
            if idx < len(sys.argv):
                address = sys.argv[idx]
            else:
                print("--address argument provided without a value.")
                sys.exit(-1)

        url_path = self.declare_parameter("url_path", "/").value
        if "--url_path" in sys.argv:
            idx = sys.argv.index("--url_path") + 1
            if idx < len(sys.argv):
                url_path = str(sys.argv[idx])
            else:
                print("--url_path argument provided without a value.")
                sys.exit(-1)

        retry_startup_delay = self.declare_parameter("retry_startup_delay", 2.0).value  # seconds.
        if "--retry_startup_delay" in sys.argv:
            idx = sys.argv.index("--retry_startup_delay") + 1
            if idx < len(sys.argv):
                retry_startup_delay = int(sys.argv[idx])
            else:
                print("--retry_startup_delay argument provided without a value.")
                sys.exit(-1)

        ##################################################
        # Done with parameter handling                   #
        ##################################################

        handlers = [(r"/", RosbridgeWebSocket), (r"", RosbridgeWebSocket)]
        if url_path != "/":
            handlers = [(rf"{url_path}", RosbridgeWebSocket)]

        application = Application(handlers, **tornado_settings)

        connected = False
        while not connected and self.context.ok():
            try:
                ssl_options = None
                if certfile is not None and keyfile is not None:
                    ssl_options = {"certfile": certfile, "keyfile": keyfile}
                sockets = bind_sockets(port, address)
                actual_port = sockets[0].getsockname()[1]
                server = HTTPServer(application, ssl_options=ssl_options)
                server.add_sockets(sockets)
                self.declare_parameter("actual_port", actual_port)
                self.get_logger().info(f"Rosbridge WebSocket server started on port {actual_port}")
                connected = True
            except OSError as e:
                self.get_logger().warn(
                    "Unable to start server: {} " "Retrying in {}s.".format(e, retry_startup_delay)
                )
                time.sleep(retry_startup_delay)

    def protocol_parameter_handling(self):
        RosbridgeWebSocket.use_compression = self.declare_parameter("use_compression", False).value

        RosbridgeWebSocket.call_services_in_new_thread = self.declare_parameter(
            "call_services_in_new_thread", True
        ).value

        RosbridgeWebSocket.default_call_service_timeout = self.declare_parameter(
            "default_call_service_timeout", 5.0
        ).value

        RosbridgeWebSocket.send_action_goals_in_new_thread = self.declare_parameter(
            "send_action_goals_in_new_thread", True
        ).value

        # get RosbridgeProtocol parameters
        RosbridgeWebSocket.fragment_timeout = self.declare_parameter(
            "fragment_timeout", RosbridgeWebSocket.fragment_timeout
        ).value

        RosbridgeWebSocket.delay_between_messages = self.declare_parameter(
            "delay_between_messages", RosbridgeWebSocket.delay_between_messages
        ).value

        RosbridgeWebSocket.max_message_size = self.declare_parameter(
            "max_message_size", RosbridgeWebSocket.max_message_size
        ).value

        RosbridgeWebSocket.unregister_timeout = self.declare_parameter(
            "unregister_timeout", RosbridgeWebSocket.unregister_timeout
        ).value

        bson_only_mode = self.declare_parameter("bson_only_mode", False).value

        RosbridgeWebSocket.client_manager = ClientManager(self)

        # Publisher for number of connected clients
        # QoS profile with transient local durability (latched topic in ROS 1).
        client_count_qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        RosbridgeWebSocket.client_count_pub = self.create_publisher(
            Int32, "client_count", qos_profile=client_count_qos_profile
        )
        RosbridgeWebSocket.client_count_pub.publish(Int32(data=0))

        # Get the glob strings and parse them as arrays.
        topics_glob = self.declare_parameter("topics_glob", "").value

        services_glob = self.declare_parameter("services_glob", "").value

        params_glob = self.declare_parameter("params_glob", "").value

        RosbridgeWebSocket.topics_glob = [
            element.strip().strip("'")
            for element in topics_glob[1:-1].split(",")
            if len(element.strip().strip("'")) > 0
        ]
        RosbridgeWebSocket.services_glob = [
            element.strip().strip("'")
            for element in services_glob[1:-1].split(",")
            if len(element.strip().strip("'")) > 0
        ]
        RosbridgeWebSocket.params_glob = [
            element.strip().strip("'")
            for element in params_glob[1:-1].split(",")
            if len(element.strip().strip("'")) > 0
        ]

        if "--fragment_timeout" in sys.argv:
            idx = sys.argv.index("--fragment_timeout") + 1
            if idx < len(sys.argv):
                RosbridgeWebSocket.fragment_timeout = int(sys.argv[idx])
            else:
                print("--fragment_timeout argument provided without a value.")
                sys.exit(-1)

        if "--delay_between_messages" in sys.argv:
            idx = sys.argv.index("--delay_between_messages") + 1
            if idx < len(sys.argv):
                RosbridgeWebSocket.delay_between_messages = float(sys.argv[idx])
            else:
                print("--delay_between_messages argument provided without a value.")
                sys.exit(-1)

        if "--max_message_size" in sys.argv:
            idx = sys.argv.index("--max_message_size") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                RosbridgeWebSocket.max_message_size = int(value)
            else:
                print("--max_message_size argument provided without a value. (can be <Integer>)")
                sys.exit(-1)

        if "--unregister_timeout" in sys.argv:
            idx = sys.argv.index("--unregister_timeout") + 1
            if idx < len(sys.argv):
                RosbridgeWebSocket.unregister_timeout = float(sys.argv[idx])
            else:
                print("--unregister_timeout argument provided without a value.")
                sys.exit(-1)

        if "--topics_glob" in sys.argv:
            idx = sys.argv.index("--topics_glob") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.topics_glob = []
                else:
                    RosbridgeWebSocket.topics_glob = [
                        element.strip().strip("'") for element in value[1:-1].split(",")
                    ]
            else:
                print("--topics_glob argument provided without a value. (can be None or a list)")
                sys.exit(-1)

        if "--services_glob" in sys.argv:
            idx = sys.argv.index("--services_glob") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.services_glob = []
                else:
                    RosbridgeWebSocket.services_glob = [
                        element.strip().strip("'") for element in value[1:-1].split(",")
                    ]
            else:
                print("--services_glob argument provided without a value. (can be None or a list)")
                sys.exit(-1)

        if "--params_glob" in sys.argv:
            idx = sys.argv.index("--params_glob") + 1
            if idx < len(sys.argv):
                value = sys.argv[idx]
                if value == "None":
                    RosbridgeWebSocket.params_glob = []
                else:
                    RosbridgeWebSocket.params_glob = [
                        element.strip().strip("'") for element in value[1:-1].split(",")
                    ]
            else:
                print("--params_glob argument provided without a value. (can be None or a list)")
                sys.exit(-1)

        if ("--bson_only_mode" in sys.argv) or bson_only_mode:
            RosbridgeWebSocket.bson_only_mode = bson_only_mode

        # To be able to access the list of topics and services, you must be able to access the rosapi services.
        if RosbridgeWebSocket.services_glob:
            RosbridgeWebSocket.services_glob.append("/rosapi/*")

        Subscribe.topics_glob = RosbridgeWebSocket.topics_glob
        Advertise.topics_glob = RosbridgeWebSocket.topics_glob
        Publish.topics_glob = RosbridgeWebSocket.topics_glob
        AdvertiseService.services_glob = RosbridgeWebSocket.services_glob
        UnadvertiseService.services_glob = RosbridgeWebSocket.services_glob
        CallService.services_glob = RosbridgeWebSocket.services_glob


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = RosbridgeWebsocketNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ros():
        executor.spin_once(timeout_sec=0.01)
        if not rclpy.ok():
            shutdown_hook()

    spin_callback = PeriodicCallback(spin_ros, 1)
    spin_callback.start()
    try:
        start_hook()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Exiting due to SIGINT")
    finally:
        shutdown_hook()  # shutdown hook to stop the server


if __name__ == "__main__":
    main()
