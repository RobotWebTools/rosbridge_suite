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

from __future__ import annotations

import argparse
import sys
import time
from typing import TYPE_CHECKING, cast

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.netutil import bind_sockets
from tornado.web import Application

from rosbridge_server import ClientManager, RosbridgeWebSocket

if TYPE_CHECKING:
    from tornado.routing import _RuleList


def start_hook() -> None:
    IOLoop.instance().start()


def shutdown_hook() -> None:
    IOLoop.instance().stop()


SERVER_PARAMETERS = (
    # Server parameters
    ("port", int, 9090, "Port to listen on for WebSocket connections."),
    ("address", str, "", "Address to bind the WebSocket server to."),
    ("url_path", str, "/", "URL path for the WebSocket server."),
    ("retry_startup_delay", float, 2.0, "Delay in seconds before retrying to start the server."),
    ("certfile", str, "", "Path to the SSL certificate file."),
    ("keyfile", str, "", "Path to the SSL key file."),
    # Tornado settings
    ("websocket_ping_interval", float, 0, "Interval in seconds for WebSocket ping messages."),
    ("websocket_ping_timeout", float, 30, "Timeout in seconds for WebSocket ping responses."),
    # Websocket handler parameters
    ("use_compression", bool, False, "Enable compression for WebSocket messages."),
)

PROTOCOL_PARAMETERS = (
    ("fragment_timeout", int, 600, "Timeout in seconds for receiving next fragment."),
    ("delay_between_messages", float, 0.0, "Delay in seconds between sending messages."),
    ("max_message_size", int, 1000000, "Maximum size of a message in bytes."),
    (
        "unregister_timeout",
        float,
        10.0,
        "How long to wait before unregistering a client from publisher after unadvertising publisher.",
    ),
    (
        "binary_encoder_type",
        str,
        "default",
        "Encoder used for encoding binary data in messages. Available: 'default', 'b64', `bson'. "
        "Ignored if bson_only_mode is True.",
    ),
    ("bson_only_mode", bool, False, "Use BSON only mode for messages."),
    ("topics_glob", str, "", "Glob patterns for topics publish/subscribe."),
    ("services_glob", str, "", "Glob patterns for services call/advertise."),
    ("actions_glob", str, "", "Glob patterns for actions send/advertise."),
    ("call_services_in_new_thread", bool, True, "Call services in a new threads."),
    ("default_call_service_timeout", float, 5.0, "Default timeout for service calls."),
    ("send_action_goals_in_new_thread", bool, True, "Send action goals in a new threads."),
)


def parse_args() -> argparse.Namespace:
    """Parse command line arguments and return them as a Namespace."""
    args = remove_ros_args(sys.argv)[1:]
    parser = argparse.ArgumentParser(description="ROS 2 Rosbridge WebSocket Server")
    for name, type_, _, description in SERVER_PARAMETERS + PROTOCOL_PARAMETERS:
        parser.add_argument(f"--{name}", type=type_, help=description)
    return parser.parse_args(args)


def parse_glob_string(glob_string: str) -> list[str] | None:
    """
    Parse a glob string into a list of patterns.

    The glob string is expected to be in the format: "['pattern1', 'pattern2']"
    """
    if not glob_string:
        return None
    if glob_string == "[]":
        return []
    # Remove the surrounding brackets and split by comma
    return [s.strip().strip("'") for s in glob_string[1:-1].split(",") if s.strip()]


class RosbridgeWebsocketNode(Node):
    def __init__(self) -> None:
        super().__init__("rosbridge_websocket")

        RosbridgeWebSocket.node_handle = self
        RosbridgeWebSocket.client_manager = ClientManager(self)

        self._handle_parameters()

        # To be able to access the list of topics and services,
        # you must be able to access the rosapi services.
        if self.protocol_parameters["services_glob"] is not None:
            self.protocol_parameters["services_glob"].append("/rosapi/*")

        RosbridgeWebSocket.protocol_parameters = self.protocol_parameters
        RosbridgeWebSocket.use_compression = self.use_compression

        self._start_server()

    def _handle_parameters(self) -> None:
        # Parse command line arguments
        args = parse_args()

        # Declare ROS parameters
        for name, _, default_value, description in SERVER_PARAMETERS + PROTOCOL_PARAMETERS:
            assert isinstance(default_value, str | int | float | bool)
            value = default_value
            if hasattr(args, name) and getattr(args, name) is not None:
                # Override the parameter with the command line argument
                value = getattr(args, name)
            self.declare_parameter(
                name, value, ParameterDescriptor(description=description, read_only=True)
            )

        # Protocol parameters
        self.protocol_parameters = {}
        for name, _, _, _ in PROTOCOL_PARAMETERS:
            self.protocol_parameters[name] = self.get_parameter(name).value

        self.protocol_parameters["topics_glob"] = parse_glob_string(
            self.protocol_parameters["topics_glob"]
        )
        self.protocol_parameters["services_glob"] = parse_glob_string(
            self.protocol_parameters["services_glob"]
        )
        self.protocol_parameters["actions_glob"] = parse_glob_string(
            self.protocol_parameters["actions_glob"]
        )

        # Server and SSL parameters
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.address = self.get_parameter("address").get_parameter_value().string_value
        self.url_path = self.get_parameter("url_path").get_parameter_value().string_value
        self.retry_startup_delay = (
            self.get_parameter("retry_startup_delay").get_parameter_value().double_value
        )
        self.certfile = self.get_parameter("certfile").get_parameter_value().string_value
        self.keyfile = self.get_parameter("keyfile").get_parameter_value().string_value

        # Tornado application parameters
        self.tornado_settings = {}
        self.tornado_settings["websocket_ping_interval"] = (
            self.get_parameter("websocket_ping_interval").get_parameter_value().double_value
        )
        self.tornado_settings["websocket_ping_timeout"] = (
            self.get_parameter("websocket_ping_timeout").get_parameter_value().double_value
        )

        # WebSocket handler parameters
        self.use_compression = (
            self.get_parameter("use_compression").get_parameter_value().bool_value
        )

    def _start_server(self) -> None:
        handlers = [(r"/", RosbridgeWebSocket), (r"", RosbridgeWebSocket)]
        if self.url_path != "/":
            handlers = [(rf"{self.url_path}", RosbridgeWebSocket)]

        application = Application(
            handlers=cast("_RuleList", handlers),
            default_host=None,
            transforms=None,
            **self.tornado_settings,
        )
        connected = False
        while not connected and self.context.ok():
            try:
                ssl_options = None
                if self.certfile and self.keyfile:
                    ssl_options = {"certfile": self.certfile, "keyfile": self.keyfile}
                sockets = bind_sockets(self.port, self.address)
                actual_port = sockets[0].getsockname()[1]
                server = HTTPServer(application, ssl_options=ssl_options)
                server.add_sockets(sockets)
                self.declare_parameter("actual_port", actual_port)
                self.get_logger().info(f"Rosbridge WebSocket server started on port {actual_port}")
                connected = True
            except OSError as e:  # noqa: PERF203
                self.get_logger().warning(
                    f"Unable to start server: {e} Retrying in {self.retry_startup_delay}s."
                )
                time.sleep(self.retry_startup_delay)


def main() -> None:
    rclpy.init()
    node = RosbridgeWebsocketNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ros() -> None:
        if not rclpy.ok():
            shutdown_hook()
            return
        executor.spin_once(timeout_sec=0.01)

    spin_callback = PeriodicCallback(spin_ros, 1)
    spin_callback.start()
    try:
        start_hook()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Exiting due to SIGINT")
    finally:
        spin_callback.stop()
        shutdown_hook()  # shutdown hook to stop the server


if __name__ == "__main__":
    main()
