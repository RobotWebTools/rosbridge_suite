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
from json import loads, dumps
import threading

from rcl_interfaces.msg import Parameter
import rclpy
from ros2param.api import call_get_parameters, call_set_parameters, get_parameter_value

""" Methods to interact with the param server.  Values have to be passed
as JSON in order to facilitate dynamically typed SRV messages """

# rospy parameter server isn't thread-safe
param_server_lock = threading.RLock()
_node = None

_parameter_type_mapping = ['', 'bool_value', 'integer_value', 'double_value', 'string_value', 'byte_array_value'
                           'bool_array_value', 'integer_array_value', 'double_array_value', 'string_array_value']


def init():
    """
    Initializes params module with a rclpy.node.Node for further use.
    This function has to be called before any other for the module to work.
    """
    global _node
    # TODO(@jubeira): remove this node; use rosapi node with MultiThreadedExecutor or
    # async / await to prevent the service calls from blocking.
    _node = rclpy.create_node('rosapi_params')


def set_param(node_name, name, value, params_glob):
    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to set the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to set the parameter.
    d = None
    try:
        d = loads(value)
    except ValueError:
        raise Exception("Due to the type flexibility of the ROS parameter server, the value argument to set_param must be a JSON-formatted string.")
    with param_server_lock:
        parameter = Parameter()
        parameter.name = name
        parameter.value = get_parameter_value(string_value=value)
        try:
            # call_get_parameters will fail if node does not exist.
            call_set_parameters(node=_node, node_name=node_name, parameters=[parameter])
        except Exception as e:
            _node.get_logger().info('Exception: {}'.format(e))
            pass


def get_param(node_name, name, default, params_glob):
    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to get the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to get the parameter.
    d = None
    if default is not "":
        try:
            d = loads(default)
        except ValueError:
            d = default
    with param_server_lock:
        try:
            # call_get_parameters will fail if node does not exist.
            response = call_get_parameters(
                node=_node, node_name=node_name,
                parameter_names=[name])
            pvalue = response.values[0]
            # if type is 0 (parameter not set), the next line will raise an exception
            # and return value shall go to default.
            value = getattr(pvalue, _parameter_type_mapping[pvalue.type])
        except:
            # If either the node or the parameter does not exist, return default.
            value = default

    return dumps(value)


def has_param(node_name, name, params_glob):
    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to set the parameter.
        return False
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, check whether the parameter exists.
    with param_server_lock:
        try:
            response = call_get_parameters(
                node=_node, node_name=node_name,
                parameter_names=[name])
        except:
            return False

    return response.values[0].type > 0 and response.values[0].type < len(_parameter_type_mapping)


# TODO(@jubeira): functions to be ported below.
def delete_param(node_name, name, params_glob):
    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to delete the parameter.
        return
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to delete the parameter.
    if has_param(name, params_glob):
        with param_server_lock:
            rospy.delete_param(name)


def search_param(name, params_glob):
    if params_glob and not any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        # If the glob list is not empty and there are no glob matches,
        # stop the attempt to find the parameter.
        return None
    # If the glob list is empty (i.e. false) or the parameter matches
    # one of the glob strings, continue to find the parameter.
    return rospy.search_param(name)


def get_param_names(params_glob):
    with param_server_lock:
        if params_glob:
            # If there is a parameter glob, filter by it.
            return filter(lambda x: any(fnmatch.fnmatch(str(x), glob) for glob in params_glob), rospy.get_param_names())
        else:
            # If there is no parameter glob, don't filter.
            return rospy.get_param_names()

