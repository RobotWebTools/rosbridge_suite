import time
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

import roslib
import rospy

from threading import Lock

""" ros_loader contains methods for dynamically loading ROS message classes at
runtime.  It's achieved by using roslib to load the manifest files for the
package that the respective class is contained in.

Methods typically return the requested class or instance, or None if not found
"""

# Variable containing the loaded classes
_loaded_msgs = {}
_loaded_srvs = {}
_msgs_lock = Lock()
_srvs_lock = Lock()
_manifest_lock = Lock()


class InvalidTypeStringException(Exception):
    def __init__(self, typestring):
        Exception.__init__(self, "%s is not a valid type string" % typestring)


class InvalidPackageException(Exception):
    def __init__(self, package, original_exception):
        Exception.__init__(self,
           "Unable to load the manifest for package %s. Caused by: %s"
           % (package, str(original_exception))
       )


class InvalidModuleException(Exception):
    def __init__(self, modname, subname, original_exception):
        Exception.__init__(self,
           "Unable to import %s.%s from package %s. Caused by: %s"
           % (modname, subname, modname, str(original_exception))
        )


class InvalidClassException(Exception):
    def __init__(self, modname, subname, classname, original_exception):
        Exception.__init__(self,
           "Unable to import %s class %s from package %s. Caused by %s"
           % (subname, classname, modname, str(original_exception))
        )


def get_message_class(typestring):
    """ Loads the message type specified.

    Returns the loaded class, or throws exceptions on failure """
    return _get_msg_class(typestring)


def get_service_class(typestring):
    """ Loads the service type specified.

    Returns the loaded class, or None on failure """
    return _get_srv_class(typestring)


def get_message_instance(typestring):
    """ If not loaded, loads the specified type.
    Then returns an instance of it, or None. """
    cls = get_message_class(typestring)
    return cls()


def get_service_instance(typestring):
    """ If not loaded, loads the specified type.
    Then returns an instance of it, or None. """
    cls = get_service_class(typestring)
    return cls()


def get_service_request_instance(typestring):
    cls = get_service_class(typestring)
    return cls._request_class()


def get_service_response_instance(typestring):
    cls = get_service_class(typestring)
    return cls._response_class()


def _get_msg_class(typestring):
    """ If not loaded, loads the specified msg class then returns an instance
    of it

    Throws various exceptions if loading the msg class fails """
    global _loaded_msgs, _msgs_lock
    return _get_class(typestring, "msg", _loaded_msgs, _msgs_lock)


def _get_srv_class(typestring):
    """ If not loaded, loads the specified srv class then returns an instance
    of it

    Throws various exceptions if loading the srv class fails """
    global _loaded_srvs, _srvs_lock
    return _get_class(typestring, "srv", _loaded_srvs, _srvs_lock)


def _get_class(typestring, subname, cache, lock):
    """ If not loaded, loads the specified class then returns an instance
    of it.

    Loaded classes are cached in the provided cache dict

    Throws various exceptions if loading the msg class fails """

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


def _load_class(modname, subname, classname):
    """ Loads the manifest and imports the module that contains the specified
    type.

    Logic is similar to that of roslib.message.get_message_class, but we want
    more expressive exceptions.

    Returns the loaded module, or None on failure """
    global loaded_modules

    try:
        with _manifest_lock:
            # roslib maintains a cache of loaded manifests, so no need to duplicate
            roslib.launcher.load_manifest(modname)
    except Exception as exc:
        raise InvalidPackageException(modname, exc)

    try:
        pypkg = __import__('%s.%s' % (modname, subname))
    except Exception as exc:
        raise InvalidModuleException(modname, subname, exc)

    try:
        return getattr(getattr(pypkg, subname), classname)
    except Exception as exc:
        raise InvalidClassException(modname, subname, classname, exc)


def _splittype(typestring):
    """ Split the string the / delimiter and strip out empty strings

    Performs similar logic to roslib.names.package_resource_name but is a bit
    more forgiving about excess slashes
    """
    splits = [x for x in typestring.split("/") if x]
    if len(splits) == 2:
        return splits
    raise InvalidTypeStringException(typestring)


def _add_to_cache(cache, lock, key, value):
    with lock:
        cache[key] = value


def _get_from_cache(cache, lock, key):
    """ Returns the value for the specified key from the cache.
    Locks the lock before doing anything. Returns None if key not in cache """
    ret = None
    with lock:
        if key in cache:
            ret = cache[key]
    return ret
