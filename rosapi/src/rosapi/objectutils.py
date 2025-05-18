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

import inspect
import logging
import re

from rosapi.stringify_field_types import stringify_field_types
from rosbridge_library.internal import ros_loader

# Keep track of atomic types and special types
atomics = [
    "bool",
    "boolean",
    "byte",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "float",
    "float32",
    "float64",
    "double",
    "string",
    "octet",
]
specials = ["time", "duration"]


def get_typedef(type):
    """A typedef is a dict containing the following fields:
         - string type
         - string[] fieldnames
         - string[] fieldtypes
         - int[] fieldarraylen
         - string[] examples
         - string[] constnames
         - string[] constvalues
    get_typedef will return a typedef dict for the specified message type"""

    # Check if the type string indicates a sequence (array) type
    if matches := re.findall("sequence<([^<]+)>", type):
        # Extract the inner type and continue processing
        type = matches[0]

    if type in atomics:
        # Atomics don't get a typedef
        return None

    if type in specials:
        # Specials get their type def mocked up
        return _get_special_typedef(type)

    # Fetch an instance and return its typedef
    try:
        instance = ros_loader.get_message_instance(type)
        type_def = _get_typedef(instance)
        return type_def
    except (ros_loader.InvalidModuleException, ros_loader.InvalidClassException) as e:
        logging.error(f"An error occurred trying to get the type definition for {type}: {e}")
        return None


def get_service_request_typedef(servicetype):
    """Returns a typedef dict for the service request class for the specified service type"""
    # Get an instance of the service request class and return its typedef
    instance = ros_loader.get_service_request_instance(servicetype)
    return _get_typedef(instance)


def get_service_response_typedef(servicetype):
    """Returns a typedef dict for the service response class for the specified service type"""
    # Get an instance of the service response class and return its typedef
    instance = ros_loader.get_service_response_instance(servicetype)
    return _get_typedef(instance)


def get_typedef_recursive(type):
    """Returns a list of typedef dicts for this type and all contained type fields"""
    # Just go straight into the recursive method
    return _get_typedefs_recursive(type, [])


def get_service_request_typedef_recursive(servicetype):
    """Returns a list of typedef dicts for this type and all contained type fields"""
    # Get an instance of the service request class and get its typedef
    instance = ros_loader.get_service_request_instance(servicetype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])


def get_service_response_typedef_recursive(servicetype):
    """Returns a list of typedef dicts for this type and all contained type fields"""
    # Get an instance of the service response class and get its typedef
    instance = ros_loader.get_service_response_instance(servicetype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])


def get_action_goal_typedef_recursive(actiontype):
    """Returns a list of typedef dicts for this type and all contained type fields"""
    # Get an instance of the action goal class and get its typedef
    instance = ros_loader.get_action_goal_instance(actiontype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])


def get_action_result_typedef_recursive(actiontype):
    """Returns a list of typedef dicts for this type and all contained type fields"""
    # Get an instance of the action result class and get its typedef
    instance = ros_loader.get_action_result_instance(actiontype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])


def get_action_feedback_typedef_recursive(actiontype):
    """Returns a list of typedef dicts for this type and all contained type fields"""
    # Get an instance of the action feedback class and get its typedef
    instance = ros_loader.get_action_feedback_instance(actiontype)
    typedef = _get_typedef(instance)

    # Return the list of sub-typedefs
    return _get_subtypedefs_recursive(typedef, [])


def get_typedef_full_text(ty):
    """Returns the full text (similar to `gendeps --cat`) for the specified message type"""
    try:
        return stringify_field_types(ty)
    except Exception as e:
        return f"# failed to get full definition text for {ty}: {str(e)}"


def _get_typedef(instance):
    """Gets a typedef dict for the specified instance"""
    if _valid_instance(instance):
        fieldnames, fieldtypes, fieldarraylen, examples = _handle_array_information(instance)
        constnames, constvalues = _handle_constant_information(instance)
        typedef = _build_typedef_dictionary(
            instance, fieldnames, fieldtypes, fieldarraylen, examples, constnames, constvalues
        )
        return typedef


def _valid_instance(instance):
    """Check if instance is valid i.e.,
    not None, has __slots__ and _fields_and_field_types attributes"""
    return not (
        instance is None
        or not hasattr(instance, "__slots__")
        or not hasattr(instance, "_fields_and_field_types")
    )


def _handle_array_information(instance):
    """Handles extraction of array information including field names, types,
    lengths and examples"""
    fieldnames = []
    fieldtypes = []
    fieldarraylen = []
    examples = []
    for slot in instance.__slots__:
        key = slot[1:] if slot.startswith("_") else slot
        if key not in instance._fields_and_field_types:
            continue

        fieldnames.append(key)
        field_type, arraylen = _handle_type_and_array_len(instance, slot)
        fieldarraylen.append(arraylen)

        value = getattr(instance, slot)
        fieldtypes.append(_type_name(field_type, value))
        examples.append(str(_handle_example(arraylen, field_type, value)))

    return fieldnames, fieldtypes, fieldarraylen, examples


def _handle_type_and_array_len(instance, name):
    """Extracts field type and determines its length if it's an array"""

    # Get original field type using instance's _fields_and_field_types property
    field_type = instance._fields_and_field_types[name[1:]]

    # Initialize arraylen
    arraylen = -1

    # If field_type is a sequence, update the `field_type` variable.
    if matches := re.findall("sequence<([^<]+)>", field_type):
        # Extract the inner type and continue processing
        field_type = matches[0]
        arraylen = 0
    else:
        if field_type[-1:] == "]":
            if field_type[-2:-1] == "[":
                arraylen = 0
                field_type = field_type[:-2]
            else:
                split = field_type.find("[")
                arraylen = int(field_type[split + 1 : -1])
                field_type = field_type[:split]

    return field_type, arraylen


def _handle_example(arraylen, field_type, field_instance):
    """Determines the example of a field instance, whether it's an array or atomic type"""
    if arraylen >= 0:
        example = []
    elif field_type not in atomics:
        example = {}
    else:
        example = field_instance
    return example


def _handle_constant_information(instance):
    """Handles extraction of constants information including constant names and values"""
    constnames = []
    constvalues = []
    attributes = inspect.getmembers(instance)
    for attribute in attributes:
        if (
            attribute[0] not in instance.__slots__
            and not attribute[0].startswith("_")
            and not inspect.isroutine(attribute[1])
        ):
            constnames.append(str(attribute[0]))
            constvalues.append(str(attribute[1]))
    return constnames, constvalues


def _build_typedef_dictionary(
    instance, fieldnames, fieldtypes, fieldarraylen, examples, constnames, constvalues
):
    """Builds the typedef dictionary from multiple inputs collected from instance"""
    typedef = {
        "type": _type_name_from_instance(instance),
        "fieldnames": fieldnames,
        "fieldtypes": fieldtypes,
        "fieldarraylen": fieldarraylen,
        "examples": examples,
        "constnames": constnames,
        "constvalues": constvalues,
    }
    return typedef


def _get_special_typedef(type):
    example = None
    if type == "time" or type == "duration":
        example = {
            "type": type,
            "fieldnames": ["secs", "nsecs"],
            "fieldtypes": ["int32", "int32"],
            "fieldarraylen": [-1, -1],
            "examples": ["0", "0"],
            "constnames": [],
            "constvalues": [],
        }
    return example


def _get_typedefs_recursive(type, typesseen):
    """returns the type def for this type as well as the type defs for any fields within the type"""
    if type in typesseen:
        # Don't put a type if it's already been seen
        return []

    # Note that we have now seen this type
    typesseen.append(type)

    # Get the typedef for this type and make sure it's not None
    typedef = get_typedef(type)

    return _get_subtypedefs_recursive(typedef, typesseen)


def _get_subtypedefs_recursive(typedef, typesseen):
    if typedef is None:
        return []

    # Create the list of subtypes and get the typedefs for fields
    typedefs = [typedef]
    for fieldtype in typedef["fieldtypes"]:
        typedefs = typedefs + _get_typedefs_recursive(fieldtype, typesseen)

    return typedefs


def _type_name(type, instance):
    """given a short type, and an object instance of that type,
    determines and returns the fully qualified type"""
    # The fully qualified type of atomic and special types is just their original name
    if type in atomics or type in specials:
        return type

    # If the instance is a list, then we can get no more information from the instance.
    # However, luckily, the 'type' field for list types is usually already inflated to the full type.
    if isinstance(instance, list):
        return type

    # Otherwise, the type will come from the module and class name of the instance
    return _type_name_from_instance(instance)


def _type_name_from_instance(instance):
    mod = instance.__module__
    type = mod[0 : mod.find(".")] + "/" + instance.__class__.__name__
    return type
