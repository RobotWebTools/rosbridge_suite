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

from threading import Thread

import rclpy
from rclpy.expand_topic_name import expand_topic_name
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_service_class,
    get_service_request_instance,
)


class InvalidServiceException(Exception):
    def __init__(self, servicename):
        Exception.__init__(self, "Service %s does not exist" % servicename)


class ServiceCaller(Thread):
    def __init__(self, service, args, success_callback, error_callback, node_handle, spin_rate=0):
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
        spin_rate        -- if nonzero, puts sleeps between executor spins
        """
        Thread.__init__(self)
        self.daemon = True
        self.service = service
        self.args = args
        self.success = success_callback
        self.error = error_callback
        self.node_handle = node_handle
        self.spin_rate = spin_rate

    def run(self):
        try:
            # Call the service and pass the result to the success handler
            self.success(
                call_service(
                    self.node_handle, self.service, args=self.args, spin_rate=self.spin_rate
                )
            )
        except Exception as e:
            # On error, just pass the exception to the error handler
            self.error(e)


def args_to_service_request_instance(service, inst, args):
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


def call_service(node_handle, service, args=None, spin_rate=0):
    # Given the service name, fetch the type and class of the service,
    # and a request instance
    service = expand_topic_name(service, node_handle.get_name(), node_handle.get_namespace())

    service_names_and_types = dict(node_handle.get_service_names_and_types())
    service_type = service_names_and_types.get(service)
    if service_type is None:
        raise InvalidServiceException(service)
    # service_type is a tuple of types at this point; only one type is supported.
    if len(service_type) > 1:
        node_handle.get_logger().warning(f"More than one service type detected: {service_type}")
    service_type = service_type[0]

    service_class = get_service_class(service_type)
    inst = get_service_request_instance(service_type)

    # Populate the instance with the provided args
    args_to_service_request_instance(service, inst, args)

    client = node_handle.create_client(service_class, service)

    if spin_rate > 0:
        future = client.call_async(inst)
        while not future.done():
            if node_handle.executor:
                node_handle.executor.spin_once(timeout_sec=spin_rate)
            else:
                rclpy.spin_once(node_handle, timeout_sec=spin_rate)
        result = future.result()
    else:
        result = client.call(inst)

    node_handle.destroy_client(client)
    if result is not None:
        # Turn the response into JSON and pass to the callback
        json_response = extract_values(result)
    else:
        raise Exception(result)

    return json_response
