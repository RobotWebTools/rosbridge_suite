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

import rospy
import sys

from socket import error
from twisted.internet import reactor
from rosbridge_server import RosbridgeUdpSocket
def shutdown_hook():
    pass
if __name__ == "__main__":
    rospy.init_node("rosbridge_websocket")
    rospy.on_shutdown(shutdown_hook)    # register shutdown hook to stop the server

    # SSL options
    certfile = rospy.get_param('~certfile', None)
    keyfile = rospy.get_param('~keyfile', None)
    # if authentication should be used
    RosbridgeUdpSocket.authenticate = rospy.get_param('~authenticate', False)
    port = rospy.get_param('~port', 9090)
    address = rospy.get_param('~address', "")

    if "--port" in sys.argv:
        idx = sys.argv.index("--port")+1
        if idx < len(sys.argv):
            port = int(sys.argv[idx])
        else:
            print "--port argument provided without a value."
            sys.exit(-1)
    rospy.loginfo("Rosbridge UDP server started on port %d", port)
    reactor.listenUDP(port, RosbridgeUdpSocket())
    reactor.run()
