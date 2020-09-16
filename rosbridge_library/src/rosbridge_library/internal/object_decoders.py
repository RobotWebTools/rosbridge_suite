#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2020 Maximilian Matthe, Barkhausen Institut gGmbH
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

# This file contains functions to decode specific ROS messages in an
# optimized way compared to the generic way of extracting the fields
# from the message type and the iterating through them. This way,
# speedups can be obtained.

import array

def decode_compressed_image(msg, inst):
    inst.header.frame_id = msg["header"]["frame_id"]
    inst.header.stamp.sec = msg["header"]["stamp"]["sec"]
    inst.header.stamp.nanosec = msg["header"]["stamp"]["nanosec"]
    inst.format = msg["format"]

    # explicitely cast to array.array to overcome a costly runtime
    # check within inst.data instance.
    # See also https://github.com/RobotWebTools/rosbridge_suite/pull/493#discussion_r425151919
    inst.data = array.array("B", msg["data"])
    return inst


shortcut_object_decoders = {"sensor_msgs/CompressedImage": decode_compressed_image}
