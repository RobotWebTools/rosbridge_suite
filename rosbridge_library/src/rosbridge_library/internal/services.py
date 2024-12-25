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

import time
from threading import Thread
from typing import Any, Callable, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.expand_topic_name import expand_topic_name
from rclpy.node import Node
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_service_class,
    get_service_request_instance,
)


class InvalidServiceException(Exception):
    def __init__(self, service_name) -> None:
        Exception.__init__(self, f"Service {service_name} does not exist")


class ServiceCaller(Thread):
    def __init__(
        self,
        service: str,
        args: dict,
        success_callback: Callable[[dict], None],
        error_callback: Callable[[Exception], None],
        node_handle: Node,
    ) -> None:
        """Create a service caller for the specified service.  Use start()
        to start in a separate thread or run() to run in this thread.

        Keyword arguments:
        service          -- the name of the service to call
        args             -- arguments to pass to the service.  Can be an
        ordered list, or a dict of name-value pairs.  Anything else will be
        treated as though no arguments were provided (which is still valid for
        some kinds of service)
        success_callback -- a callback to call with the JSON result of the
        service call
        error_callback   -- a callback to call if an error occurs.  The
        callback will be passed the exception that caused the failure
        node_handle      -- a ROS 2 node handle to call services.
        """
        Thread.__init__(self)
        self.daemon = True
        self.service = service
        self.args = args
        self.success = success_callback
        self.error = error_callback
        self.node_handle = node_handle

    def run(self) -> None:
        try:
            # Call the service and pass the result to the success handler
            self.success(call_service(self.node_handle, self.service, args=self.args))
        except Exception as e:
            # On error, just pass the exception to the error handler
            self.error(e)


def args_to_service_request_instance(service: str, inst: Any, args: list | dict | None) -> Any:
    """Populate a service request instance with the provided args

    args can be a dictionary of values, or a list, or None

    Propagates any exceptions that may be raised."""
    msg = {}
    if isinstance(args, list):
        msg = dict(zip(inst.get_fields_and_field_types().keys(), args))
    elif isinstance(args, dict):
        msg = args

    # Populate the provided instance, propagating any exceptions
    populate_instance(msg, inst)


def call_service(
    node_handle: Node,
    service: str,
    args: Optional[dict] = None,
    server_timeout_time: float = 1.0,
    sleep_time: float = 0.001,
) -> dict:
    # Given the service name, fetch the type and class of the service,
    # and a request instance
    service = expand_topic_name(service, node_handle.get_name(), node_handle.get_namespace())

    service_names_and_types = dict(node_handle.get_service_names_and_types())
    service_types = service_names_and_types.get(service)
    if service_types is None:
        raise InvalidServiceException(service)

    # service_type is a tuple of types at this point; only one type is supported.
    if len(service_types) > 1:
        node_handle.get_logger().warning(f"More than one service type detected: {service_types}")
    service_type = service_types[0]

    service_class = get_service_class(service_type)
    inst = get_service_request_instance(service_type)

    # Populate the instance with the provided args
    args_to_service_request_instance(service, inst, args)

    client = node_handle.create_client(
        service_class, service, callback_group=ReentrantCallbackGroup()
    )

    if not client.wait_for_service(server_timeout_time):
        node_handle.destroy_client(client)
        raise InvalidServiceException(service)

    future = client.call_async(inst)
    while rclpy.ok() and not future.done():
        time.sleep(sleep_time)
    result = future.result()

    node_handle.destroy_client(client)
    if result is not None:
        # Turn the response into JSON and pass to the callback
        json_response = extract_values(result)
    else:
        raise Exception(result)

    return json_response
