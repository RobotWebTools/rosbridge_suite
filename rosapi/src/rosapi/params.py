# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# Copyright (c) 2025, Fictionlab sp. z o.o.
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
from json import dumps, loads
from typing import TYPE_CHECKING

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
from rclpy.task import Future
from ros2node.api import get_absolute_node_name
from rosapi.async_helper import futures_wait_for
from rosapi.proxy import get_nodes

if TYPE_CHECKING:
    from rclpy.client import Client

""" Methods to interact with the param server.  Values have to be passed
as JSON in order to facilitate dynamically typed SRV messages """

# Constants
DEFAULT_PARAM_TIMEOUT_SEC = 5.0

_node = None
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


def init(node: Node, timeout_sec: float | int = DEFAULT_PARAM_TIMEOUT_SEC):
    """
    Initializes params module with a rclpy.node.Node for further use.
    This function has to be called before any other for the module to work.
    """
    global _node, _timeout_sec
    _node = node

    if not isinstance(timeout_sec, (int, float)) or timeout_sec <= 0:
        raise ValueError("Parameter timeout must be a positive number")
    _timeout_sec = timeout_sec


async def set_param(node_name: str, name: str, value: str, params_glob: list[str]):
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
    await _set_param(node_name, name, value)


async def _set_param(node_name: str, name: str, value: str, parameter_type=None):
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
            setattr(parameter.value, _parameter_type_mapping[parameter_type], loads(value))

    assert _node is not None
    client: Client = _node.create_client(
        SetParameters,
        f"{node_name}/set_parameters",
        callback_group=MutuallyExclusiveCallbackGroup(),
    )

    if not client.service_is_ready():
        _node.destroy_client(client)
        raise Exception(f"Service {client.srv_name} is not available")

    request = SetParameters.Request()
    request.parameters = [parameter]

    future = client.call_async(request)

    await futures_wait_for(_node, [future], _timeout_sec)

    _node.destroy_client(client)

    if not future.done():
        future.cancel()
        raise Exception("Timeout occurred")

    result = future.result()

    assert result is not None
    if not result.results[0].successful:
        raise Exception(result.results[0].reason)


async def get_param(node_name: str, name: str, params_glob: str) -> str:
    """Gets a parameter from a given node"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to get the parameter.
        raise Exception(f"Parameter {name} does not match any of the glob strings")
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to get the parameter.

    node_name = get_absolute_node_name(node_name)
    pvalue = await _get_param(node_name, name)
    value = getattr(pvalue, _parameter_type_mapping[pvalue.type])

    # Convert array types to lists for JSON serialization
    if hasattr(value, "tolist"):  # This will catch numpy arrays and Python arrays
        value = value.tolist()

    return dumps(value)


async def _get_param(node_name: str, name: str) -> ParameterValue:
    """Internal helper function for get_param"""

    assert _node is not None
    client: Client = _node.create_client(
        GetParameters,
        f"{node_name}/get_parameters",
        callback_group=MutuallyExclusiveCallbackGroup(),
    )

    if not client.service_is_ready():
        _node.destroy_client(client)
        raise Exception(f"Service {client.srv_name} is not available")

    request = GetParameters.Request()
    request.names = [name]

    future = client.call_async(request)

    await futures_wait_for(_node, [future], _timeout_sec)

    _node.destroy_client(client)

    if not future.done():
        future.cancel()
        raise Exception("Timeout occurred")

    result = future.result()

    assert result is not None
    if len(result.values) == 0:
        raise Exception(f"Parameter {name} not found")

    return result.values[0]


async def has_param(node_name: str, name: str, params_glob: str) -> bool:
    """Checks whether a given node has a parameter or not"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to set the parameter.
        return False
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, check whether the parameter exists.
    node_name = get_absolute_node_name(node_name)
    try:
        pvalue = await _get_param(node_name, name)
    except Exception:
        return False

    return 0 < pvalue.type < len(_parameter_type_mapping)


async def delete_param(node_name, name, params_glob):
    """Deletes a parameter in a given node"""

    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to delete the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to delete the parameter.
    node_name = get_absolute_node_name(node_name)
    if await has_param(node_name, name, params_glob):
        await _set_param(node_name, name, None, ParameterType.PARAMETER_NOT_SET)


async def get_param_names(params_glob: str | None) -> list[str]:
    assert _node is not None

    nodes = [get_absolute_node_name(node) for node in get_nodes()]

    futures: list[tuple[str, Future]] = []
    clients = []
    for node_name in nodes:
        if node_name == _node.get_fully_qualified_name():
            continue

        client: Client = _node.create_client(
            ListParameters,
            f"{node_name}/list_parameters",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        if client.service_is_ready():
            future = client.call_async(ListParameters.Request())
            futures.append((node_name, future))
            clients.append(client)
        else:
            _node.destroy_client(client)

    params = []

    await futures_wait_for(_node, [future for _, future in futures], _timeout_sec)

    for client in clients:
        _node.destroy_client(client)

    for node_name, future in futures:
        if not future.done():
            future.cancel()
            continue

        if future.exception() is not None:
            continue

        result = future.result()
        if result is not None:
            params.extend([f"{node_name}:{param_name}" for param_name in result.result.names])

    if params_glob:
        return list(
            filter(lambda x: any(fnmatch.fnmatch(str(x), glob) for glob in params_glob), params)
        )
    else:
        return params
