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

import fnmatch
import threading
from json import dumps, loads

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import ListParameters
from ros2node.api import get_absolute_node_name
from ros2param.api import call_get_parameters, call_set_parameters, get_parameter_value
from rosapi.proxy import get_nodes

""" Methods to interact with the param server.  Values have to be passed
as JSON in order to facilitate dynamically typed SRV messages """

# Constants
DEFAULT_PARAM_TIMEOUT_SEC = 5.0

# Ensure thread safety for setting / getting parameters.
param_server_lock = threading.RLock()
_node = None
_parent_node_name = ""
_timeout_sec = DEFAULT_PARAM_TIMEOUT_SEC

_parameter_type_mapping = [
    "",
    "bool_value",
    "integer_value",
    "double_value",
    "string_value",
    "byte_array_value",
    "bool_array_value",
    "integer_array_value",
    "double_array_value",
    "string_array_value",
]


def init(parent_node_name, timeout_sec=DEFAULT_PARAM_TIMEOUT_SEC):
    """
    Initializes params module with a rclpy.node.Node for further use.
    This function has to be called before any other for the module to work.
    """
    global _node, _parent_node_name, _timeout_sec
    # TODO(@jubeira): remove this node; use rosapi node with MultiThreadedExecutor or
    # async / await to prevent the service calls from blocking.
    parent_node_basename = parent_node_name.split("/")[-1]
    param_node_name = f"{parent_node_basename}_params"
    _node = rclpy.create_node(
        param_node_name,
        cli_args=["--ros-args", "-r", f"__node:={param_node_name}"],
        start_parameter_services=False,
    )
    _parent_node_name = get_absolute_node_name(parent_node_name)

    if not isinstance(timeout_sec, (int, float)) or timeout_sec <= 0:
        raise ValueError("Parameter timeout must be a positive number")
    _timeout_sec = timeout_sec


def set_param(node_name, name, value, params_glob):
    """Sets a parameter in a given node"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to set the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to set the parameter.
    d = None
    try:
        d = loads(value)
        value = d if isinstance(d, str) else value
    except ValueError:
        raise Exception(
            "Due to the type flexibility of the ROS parameter server, the value argument to set_param must be a JSON-formatted string."
        )

    node_name = get_absolute_node_name(node_name)
    with param_server_lock:
        _set_param(node_name, name, value)


def _set_param(node_name, name, value, parameter_type=None):
    """
    Internal helper function for set_param.
    Attempts to set the given parameter in the target node with the desired value,
    deducing the parameter type if it's not specified.
    parameter_type allows forcing a type for the given value; this is useful to delete parameters.
    """
    parameter = Parameter()
    parameter.name = name
    if parameter_type is None:
        parameter.value = get_parameter_value(string_value=value)
    else:
        parameter.value = ParameterValue()
        parameter.value.type = parameter_type
        if parameter_type != ParameterType.PARAMETER_NOT_SET:
            setattr(parameter.value, _parameter_type_mapping[parameter_type])

    try:
        # call_get_parameters will fail if node does not exist.
        call_set_parameters(node=_node, node_name=node_name, parameters=[parameter])
    except Exception:
        pass


def get_param(node_name, name, default, params_glob):
    """Gets a parameter from a given node"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to get the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to get the parameter.
    if default != "":
        try:
            default = loads(default)
        except ValueError:
            pass  # Keep default without modifications.

    node_name = get_absolute_node_name(node_name)
    with param_server_lock:
        try:
            # call_get_parameters will fail if node does not exist.
            response = call_get_parameters(node=_node, node_name=node_name, parameter_names=[name])
            pvalue = response.values[0]
            # if type is 0 (parameter not set), the next line will raise an exception
            # and return value shall go to default.
            value = getattr(pvalue, _parameter_type_mapping[pvalue.type])
        except Exception:
            # If either the node or the parameter does not exist, return default.
            value = default

    return dumps(value)


def has_param(node_name, name, params_glob):
    """Checks whether a given node has a parameter or not"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to set the parameter.
        return False
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, check whether the parameter exists.
    node_name = get_absolute_node_name(node_name)
    with param_server_lock:
        try:
            response = call_get_parameters(node=_node, node_name=node_name, parameter_names=[name])
        except Exception:
            return False

    return response.values[0].type > 0 and response.values[0].type < len(_parameter_type_mapping)


def delete_param(node_name, name, params_glob):
    """Deletes a parameter in a given node"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to delete the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to delete the parameter.
    node_name = get_absolute_node_name(node_name)
    if has_param(node_name, name, params_glob):
        with param_server_lock:
            _set_param(node_name, name, None, ParameterType.PARAMETER_NOT_SET)


def get_param_names(params_glob):
    params = []
    nodes = get_nodes()

    for node in nodes:
        params.extend(get_node_param_names(node, params_glob))

    return params


def get_node_param_names(node_name, params_glob):
    """Gets list of parameter names for a given node"""
    node_name = get_absolute_node_name(node_name)

    with param_server_lock:
        if params_glob:
            # If there is a parameter glob, filter by it.
            return list(
                filter(
                    lambda x: any(fnmatch.fnmatch(str(x), glob) for glob in params_glob),
                    _get_param_names(node_name),
                )
            )
        else:
            # If there is no parameter glob, don't filter.
            return _get_param_names(node_name)


def _get_param_names(node_name):
    # This method is called in a service callback; calling a service of the same node
    # will cause a deadlock.
    global _parent_node_name
    if node_name == _parent_node_name or node_name == _node.get_fully_qualified_name():
        return []

    client = _node.create_client(ListParameters, f"{node_name}/list_parameters")

    if not client.service_is_ready():
        return []

    request = ListParameters.Request()
    future = client.call_async(request)
    if _node.executor:
        _node.executor.spin_until_future_complete(future, timeout_sec=_timeout_sec)
    else:
        rclpy.spin_until_future_complete(_node, future, timeout_sec=_timeout_sec)
    response = future.result()

    if response is not None:
        return [f"{node_name}:{param_name}" for param_name in response.result.names]
    else:
        return []
