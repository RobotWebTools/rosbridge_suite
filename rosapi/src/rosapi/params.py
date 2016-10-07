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
import rospy
import threading

from json import loads, dumps


""" Methods to interact with the param server.  Values have to be passed
as JSON in order to facilitate dynamically typed SRV messages """

# rospy parameter server isn't thread-safe
param_server_lock = threading.RLock()

def set_param(name, value, params_glob):
    if any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        d = None
        try:
            d = loads(value)
        except ValueError:
            raise Exception("Due to the type flexibility of the ROS parameter server, the value argument to set_param must be a JSON-formatted string.")
        with param_server_lock:
            rospy.set_param(name, d)
    
    
def get_param(name, default, params_glob):
    if any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        d = None
        if default is not "":
            try:
                d = loads(default)
            except ValueError:
                d = default

        with param_server_lock:
            value = rospy.get_param(name, d)
        return dumps(value)
    else:
        return default

def has_param(name, params_glob):
    if any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        with param_server_lock:
            return rospy.has_param(name)
    else:
        return False


def delete_param(name, params_glob):
    if any(fnmatch.fnmatch(str(name), glob) for glob in params_glob):
        with param_server_lock:
            if has_param(name):
                rospy.delete_param(name)
        
        
def search_param(name, params_glob):
    with param_server_lock:
        v = rospy.search_param(name)
        if any(fnmatch.fnmatch(str(v), glob) for glob in params_glob):
            return v
        else:
            return None

def get_param_names(params_glob):
    with param_server_lock:
        return filter(lambda x: any(fnmatch.fnmatch(str(x), glob) for glob in params_glob), rospy.get_param_names())
