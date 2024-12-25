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

import importlib
from threading import Lock
from typing import Any, Dict, Tuple

""" ros_loader contains methods for dynamically loading ROS message classes at
runtime.  It's achieved by using roslib to load the manifest files for the
package that the respective class is contained in.

Methods typically return the requested class or instance, or None if not found
"""

# Variable containing the loaded classes
_loaded_msgs: dict[str, Any] = {}
_loaded_srvs: dict[str, Any] = {}
_loaded_actions: dict[str, Any] = {}
_msgs_lock = Lock()
_srvs_lock = Lock()
_actions_lock = Lock()


class InvalidTypeStringException(Exception):
    def __init__(self, typestring: str) -> None:
        Exception.__init__(self, f"{typestring} is not a valid type string")


class InvalidModuleException(Exception):
    def __init__(self, modname: str, subname: str, original_exception: Exception) -> None:
        Exception.__init__(
            self,
            "Unable to import %s.%s from package %s. Caused by: %s"
            % (modname, subname, modname, str(original_exception)),
        )


class InvalidClassException(Exception):
    def __init__(
        self, modname: str, subname: str, classname: str, original_exception: Exception
    ) -> None:
        Exception.__init__(
            self,
            "Unable to import %s class %s from package %s. Caused by %s"
            % (subname, classname, modname, str(original_exception)),
        )


def get_message_class(typestring: str) -> Any:
    """Loads the message type specified.

    Returns the loaded class, or throws exceptions on failure"""
    return _get_interface_class(typestring, "msg", _loaded_msgs, _msgs_lock)


def get_service_class(typestring: str) -> Any:
    """Loads the service type specified.

    Returns the loaded class, or None on failure"""
    return _get_interface_class(typestring, "srv", _loaded_srvs, _srvs_lock)


def get_action_class(typestring: str) -> Any:
    """Loads the action type specified.
    Returns the loaded class, or throws exceptions on failure"""
    return _get_interface_class(typestring, "action", _loaded_actions, _actions_lock)


def get_message_instance(typestring: str) -> Any:
    """If not loaded, loads the specified type.
    Then returns an instance of it, or None."""
    cls = get_message_class(typestring)
    return cls()


def get_service_request_instance(typestring: str) -> Any:
    cls = get_service_class(typestring)
    return cls.Request()


def get_service_response_instance(typestring: str) -> Any:
    cls = get_service_class(typestring)
    return cls.Response()


def get_action_goal_instance(typestring: str) -> Any:
    cls = get_action_class(typestring)
    return cls.Goal()


def get_action_feedback_instance(typestring: str) -> Any:
    cls = get_action_class(typestring)
    return cls.Feedback()


def get_action_result_instance(typestring: str) -> Any:
    cls = get_action_class(typestring)
    return cls.Result()


def _get_interface_class(
    typestring: str, intf_type: str, loaded_intfs: Dict[str, Any], intf_lock: Lock
) -> Any:
    """
    If not loaded, loads the specified ROS interface class then returns an instance of it.

    Throws various exceptions if loading the interface class fails.
    """
    try:
        # The type string starts with the package and ends with the
        # class and contains module subnames in between. For
        # compatibility with ROS 1 style types, we fall back to use a
        # standard "msg" subname.
        splits = [x for x in typestring.split("/") if x]
        if len(splits) > 2:
            subname = ".".join(splits[1:-1])
        else:
            subname = intf_type

        return _get_class(typestring, subname, loaded_intfs, intf_lock)
    except (InvalidModuleException, InvalidClassException):
        return _get_class(typestring, intf_type, loaded_intfs, intf_lock)


def _get_class(typestring: str, subname: str, cache: Dict[str, Any], lock: Lock) -> Any:
    """If not loaded, loads the specified class then returns an instance
    of it.

    Loaded classes are cached in the provided cache dict

    Throws various exceptions if loading the msg class fails.
    """

    # First, see if we have this type string cached
    cls = _get_from_cache(cache, lock, typestring)
    if cls is not None:
        return cls

    # Now normalise the typestring
    modname, classname = _splittype(typestring)
    norm_typestring = modname + "/" + classname

    # Check to see if the normalised type string is cached
    cls = _get_from_cache(cache, lock, norm_typestring)
    if cls is not None:
        return cls

    # Load the class
    cls = _load_class(modname, subname, classname)

    # Cache the class for both the regular and normalised typestring
    _add_to_cache(cache, lock, typestring, cls)
    _add_to_cache(cache, lock, norm_typestring, cls)

    return cls


def _load_class(modname: str, subname: str, classname: str) -> Any:
    """Loads the manifest and imports the module that contains the specified
    type.

    Logic is similar to that of roslib.message.get_message_class, but we want
    more expressive exceptions.

    Returns the loaded module, or None on failure"""

    # This assumes the module is already in the path.
    try:
        pypkg = importlib.import_module(f"{modname}.{subname}")
    except Exception as exc:
        raise InvalidModuleException(modname, subname, exc)

    try:
        return getattr(pypkg, classname)
    except Exception as exc:
        raise InvalidClassException(modname, subname, classname, exc)


def _splittype(typestring: str) -> Tuple[str, str]:
    """Split the string the / delimiter and strip out empty strings

    Performs similar logic to roslib.names.package_resource_name but is a bit
    more forgiving about excess slashes
    """
    splits = [x for x in typestring.split("/") if x]
    if len(splits) == 3:
        return (splits[0], splits[2])
    if len(splits) == 2:
        return (splits[0], splits[1])
    raise InvalidTypeStringException(typestring)


def _add_to_cache(cache: Dict[str, Any], lock: Lock, key: str, value: Any) -> None:
    lock.acquire()
    cache[key] = value
    lock.release()


def _get_from_cache(cache: Dict[str, Any], lock: Lock, key: str) -> Any:
    """Returns the value for the specified key from the cache.
    Locks the lock before doing anything. Returns None if key not in cache"""
    lock.acquire()
    ret = None
    if key in cache:
        ret = cache[key]
    lock.release()
    return ret
