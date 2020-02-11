#!/usr/bin/env python
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

from __future__ import print_function
import rclpy
from rclpy.clock import ROSClock

from rosbridge_library.internal import ros_loader

import math
import re
import string
from base64 import standard_b64encode, standard_b64decode

from rosbridge_library.util import string_types, bson

import sys
if sys.version_info >= (3, 0):
    type_map = {
    "bool":    ["bool"],
    "int":     ["int8", "byte", "uint8", "char",
                "int16", "uint16", "int32", "uint32",
                "int64", "uint64", "float32", "float64"],
    "float":   ["float32", "float64", "double"],
    "str":     ["string"]
    }
    primitive_types = [bool, int, float]
    python2 = False
else:
    type_map = {
    "bool":    ["bool"],
    "int":     ["int8", "byte", "uint8", "char",
                "int16", "uint16", "int32", "uint32",
                "int64", "uint64", "float32", "float64"],
    "float":   ["float32", "float64"],
    "str":     ["string"],
    "unicode": ["string"],
    "long":    ["int64", "uint64"]
    }
    primitive_types = [bool, int, long, float]
    python2 = True

list_types = [list, tuple]
ros_time_types = ["builtin_interfaces/Time", "builtin_interfaces/Duration"]
ros_primitive_types = ["bool", "byte", "char", "int8", "uint8", "int16",
                       "uint16", "int32", "uint32", "int64", "uint64",
                       "float32", "float64", "double", "string"]
ros_header_types = ["Header", "std_msgs/Header", "roslib/Header"]
ros_binary_types = ["uint8[]", "char[]"]
list_tokens = re.compile('<(.+?)>')
ros_binary_types_list_braces = [("uint8[]", re.compile(r'uint8\[[^\]]*\]')),
                                ("char[]", re.compile(r'char\[[^\]]*\]'))]

binary_encoder = None
binary_encoder_type = 'default'
bson_only_mode = False

# TODO(@jubeira): configure module with a node handle.
# The original code doesn't seem to actually use these parameters.
def configure(node_handle=None):
    global binary_encoder, binary_encoder_type, bson_only_mode

    if node_handle is not None:
        binary_encoder_type = node_handler.get_parameter_or('binary_encoder',
            Parameter('', value='default')).value
        bson_only_mode = node_handler.get_parameter_or('bson_only_mode',
            Parameter('', value=False)).value

    if binary_encoder is None:
        if binary_encoder_type == 'bson' or bson_only_mode:
            binary_encoder = bson.Binary
        elif binary_encoder_type == 'default' or binary_encoder_type == 'b64':
             binary_encoder = standard_b64encode
        else:
            print("Unknown encoder type '%s'"%binary_encoder_type)
            exit(0)

def get_encoder():
    configure()
    return binary_encoder

class InvalidMessageException(Exception):
    def __init__(self, inst):
        Exception.__init__(self, "Unable to extract message values from %s instance" % type(inst).__name__)


class NonexistentFieldException(Exception):
    def __init__(self, basetype, fields):
        Exception.__init__(self, "Message type %s does not have a field %s" % (basetype, '.'.join(fields)))


class FieldTypeMismatchException(Exception):
    def __init__(self, roottype, fields, expected_type, found_type):
        if roottype == expected_type:
            Exception.__init__(self, "Expected a JSON object for type %s but received a %s" % (roottype, found_type))
        else:
            Exception.__init__(self, "%s message requires a %s for field %s, but got a %s" % (roottype, expected_type, '.'.join(fields), found_type))


def extract_values(inst):
    rostype = msg_instance_type_repr(inst)
    if rostype is None:
        raise InvalidMessageException(inst=inst)
    return _from_inst(inst, rostype)


def populate_instance(msg, inst):
    """ Returns an instance of the provided class, with its fields populated
    according to the values in msg """
    inst_type = msg_instance_type_repr(inst)

    return _to_inst(msg, inst_type, inst_type, inst)


def msg_instance_type_repr(msg_inst):
    """Returns a string representation of a ROS2 message type from a message instance"""
    # Message representation: '{package}.msg.{message_name}({fields})'.
    # A representation like '_type' member in ROS1 messages is needed: '{package}/{message_name}'.
    # E.g: 'std_msgs/Header'
    msg_type = type(msg_inst)
    if msg_type in primitive_types or msg_type in list_types:
        return str(type(inst))
    inst_repr = str(msg_inst).split('.')
    return '{}/{}'.format(inst_repr[0], inst_repr[2].split('(')[0])


def msg_class_type_repr(msg_class):
    """Returns a string representation of a ROS2 message type from a class representation."""
    # The string representation of the class is <class '{package}.msg._{message}.{Message}'>
    # (e.g. <class 'std_msgs.msg._string.String'>).
    # This has to be converted to {package}/msg/{Message} (e.g. std_msgs/msg/String).
    class_repr = str(msg_class).split('\'')[1].split('.')
    return '{}/{}/{}'.format(class_repr[0], class_repr[1], class_repr[3])


def _from_inst(inst, rostype):
    global bson_only_mode
    # Special case for uint8[], we encode the string
    for binary_type, expression in ros_binary_types_list_braces:
        if expression.sub(binary_type, rostype) in ros_binary_types:
            encoded = get_encoder()(inst)
            return encoded if python2 else encoded.decode('ascii')

    # Check for time or duration
    if rostype in ros_time_types:
        return {"secs": inst.secs, "nsecs": inst.nsecs}

    if(bson_only_mode is None):bson_only_mode = rospy.get_param('~bson_only_mode', False)
    # Check for primitive types
    if rostype in ros_primitive_types:
        #JSON does not support Inf and NaN. They are mapped to None and encoded as null
        if (not bson_only_mode) and (rostype in type_map.get('float')):
            if math.isnan(inst) or math.isinf(inst):
                return None
        return inst

    # Check if it's a list or tuple
    if type(inst) in list_types:
        return _from_list_inst(inst, rostype)

    # Assume it's otherwise a full ros msg object
    return _from_object_inst(inst, rostype)


def _from_list_inst(inst, rostype):
    # Can duck out early if the list is empty
    if len(inst) == 0:
        return []

    # Remove the list indicators from the rostype
    rostype = re.search(list_tokens, rostype).group(1)

    # Shortcut for primitives
    if rostype in ros_primitive_types and not rostype in type_map.get('float'):
        return list(inst)

    # Call to _to_inst for every element of the list
    return [_from_inst(x, rostype) for x in inst]


def _from_object_inst(inst, rostype):
    # Create an empty dict then populate with values from the inst
    msg = {}
    # Equivalent for zip(inst.__slots__, inst._slot_types) in ROS1:
    for field_name, field_rostype in inst.get_fields_and_field_types().items():
        field_inst = getattr(inst, field_name)
        msg[field_name] = _from_inst(field_inst, field_rostype)
    return msg


def _to_inst(msg, rostype, roottype, inst=None, stack=[]):
    # Check if it's uint8[], and if it's a string, try to b64decode
    for binary_type, expression in ros_binary_types_list_braces:
        if expression.sub(binary_type, rostype) in ros_binary_types:
            return _to_binary_inst(msg)

    # Check the type for time or rostime
    if rostype in ros_time_types:
        return _to_time_inst(msg, rostype, inst)

    # Check to see whether this is a primitive type
    if rostype in ros_primitive_types:
        return _to_primitive_inst(msg, rostype, roottype, stack)

    # Check whether we're dealing with a list type
    if inst is not None and type(inst) in list_types:
        return _to_list_inst(msg, rostype, roottype, inst, stack)

    # Otherwise, the type has to be a full ros msg type, so msg must be a dict
    if inst is None:
        inst = ros_loader.get_message_instance(rostype)

    return _to_object_inst(msg, rostype, roottype, inst, stack)


def _to_binary_inst(msg):
    if type(msg) in string_types:
        try:
            return standard_b64decode(msg)
        except :
            return msg
    else:
        try:
            return bytes(bytearray(msg))
        except:
            return msg


def _to_time_inst(msg, rostype, inst=None):
    # Create an instance if we haven't been provided with one

    if rostype == "time" and msg == "now":
        return ROSClock().now().to_msg()

    if inst is None:
        if rostype == "time":
            inst = rclpy.time.Time().to_msg()
        elif rostype == "duration":
            inst = rclpy.duration.Duration().to_msg()
        else:
            return None

    # Copy across the fields
    for field in ["secs", "nsecs"]:
        try:
            if field in msg:
                setattr(inst, field, msg[field])
        except TypeError:
            continue

    return inst


def _to_primitive_inst(msg, rostype, roottype, stack):
    # Typecheck the msg
    msgtype = type(msg)
    if msgtype in primitive_types and rostype in type_map[msgtype.__name__]:
        return msg
    elif msgtype in string_types and rostype in type_map[msgtype.__name__]:
        return msg.encode("utf-8", "ignore") if python2 else msg
    raise FieldTypeMismatchException(roottype, stack, rostype, msgtype)


def _to_list_inst(msg, rostype, roottype, inst, stack):
    # Typecheck the msg
    if type(msg) not in list_types:
        raise FieldTypeMismatchException(roottype, stack, rostype, type(msg))

    # Can duck out early if the list is empty
    if len(msg) == 0:
        return []

    # Remove the list indicators from the rostype
    rostype = re.search(list_tokens, rostype).group(1)

    # Call to _to_inst for every element of the list
    return [_to_inst(x, rostype, roottype, None, stack) for x in msg]


def _to_object_inst(msg, rostype, roottype, inst, stack):
    # Typecheck the msg
    if type(msg) is not dict:
        raise FieldTypeMismatchException(roottype, stack, rostype, type(msg))

    # Substitute the correct time if we're an std_msgs/Header
    if rostype in ros_header_types:
        inst.stamp = ROSClock().now().to_msg()

    inst_fields = inst.get_fields_and_field_types()

    for field_name in msg:
        # Add this field to the field stack
        field_stack = stack + [field_name]

        # Raise an exception if the msg contains a bad field
        if not field_name in inst_fields:
            raise NonexistentFieldException(roottype, field_stack)

        field_rostype = inst_fields[field_name]
        field_inst = getattr(inst, field_name)

        field_value = _to_inst(msg[field_name], field_rostype,
                    roottype, field_inst, field_stack)

        setattr(inst, field_name, field_value)

    return inst
