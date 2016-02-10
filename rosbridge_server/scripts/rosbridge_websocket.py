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

from tornado.ioloop import IOLoop
from tornado.ioloop import PeriodicCallback
from tornado.web import Application

from rosbridge_server import RosbridgeWebSocket


def shutdown_hook():
    IOLoop.instance().stop()

if __name__ == "__main__":
    rospy.init_node("rosbridge_websocket")
    rospy.on_shutdown(shutdown_hook)    # register shutdown hook to stop the server

    ##################################################
    # Parameter handling                             #
    ##################################################
    retry_startup_delay = rospy.get_param('~retry_startup_delay', 2.0)  # seconds

    # get RosbridgeProtocol parameters
    RosbridgeWebSocket.fragment_timeout = rospy.get_param('~fragment_timeout',
                                                          RosbridgeWebSocket.fragment_timeout)
    RosbridgeWebSocket.delay_between_messages = rospy.get_param('~delay_between_messages',
                                                                RosbridgeWebSocket.delay_between_messages)
    RosbridgeWebSocket.max_message_size = rospy.get_param('~max_message_size',
                                                          RosbridgeWebSocket.max_message_size)
    if RosbridgeWebSocket.max_message_size == "None":
        RosbridgeWebSocket.max_message_size = None

    # SSL options
    certfile = rospy.get_param('~certfile', None)
    keyfile = rospy.get_param('~keyfile', None)
    # if authentication should be used
    RosbridgeWebSocket.authenticate = rospy.get_param('~authenticate', False)
    port = rospy.get_param('~port', 9090)
    address = rospy.get_param('~address', "")

    if "--port" in sys.argv:
        idx = sys.argv.index("--port")+1
        if idx < len(sys.argv):
            port = int(sys.argv[idx])
        else:
            print "--port argument provided without a value."
            sys.exit(-1)

    if "--address" in sys.argv:
        idx = sys.argv.index("--address")+1
        if idx < len(sys.argv):
            address = int(sys.argv[idx])
        else:
            print "--address argument provided without a value."
            sys.exit(-1)

    if "--retry_startup_delay" in sys.argv:
        idx = sys.argv.index("--retry_startup_delay") + 1
        if idx < len(sys.argv):
            retry_startup_delay = int(sys.argv[idx])
        else:
            print "--retry_startup_delay argument provided without a value."
            sys.exit(-1)

    if "--fragment_timeout" in sys.argv:
        idx = sys.argv.index("--fragment_timeout") + 1
        if idx < len(sys.argv):
            RosbridgeWebSocket.fragment_timeout = int(sys.argv[idx])
        else:
            print "--fragment_timeout argument provided without a value."
            sys.exit(-1)

    if "--delay_between_messages" in sys.argv:
        idx = sys.argv.index("--delay_between_messages") + 1
        if idx < len(sys.argv):
            RosbridgeWebSocket.delay_between_messages = float(sys.argv[idx])
        else:
            print "--delay_between_messages argument provided without a value."
            sys.exit(-1)

    if "--max_message_size" in sys.argv:
        idx = sys.argv.index("--max_message_size") + 1
        if idx < len(sys.argv):
            value = sys.argv[idx]
            if value == "None":
                RosbridgeWebSocket.max_message_size = None
            else:
                RosbridgeWebSocket.max_message_size = int(value)
        else:
            print "--max_message_size argument provided without a value. (can be None or <Integer>)"
            sys.exit(-1)

    ##################################################
    # Done with parameter handling                   #
    ##################################################

    application = Application([(r"/", RosbridgeWebSocket), (r"", RosbridgeWebSocket)])

    connected = False
    while not connected and not rospy.is_shutdown():
        try:
            if certfile is not None and keyfile is not None:
                application.listen(port, address, ssl_options={ "certfile": certfile, "keyfile": keyfile})
            else:
                application.listen(port, address)
            rospy.loginfo("Rosbridge WebSocket server started on port %d", port)
            connected = True
        except error as e:
            rospy.logwarn("Unable to start server: " + str(e) +
                          " Retrying in " + str(retry_startup_delay) + "s.")
            rospy.sleep(retry_startup_delay)

    IOLoop.instance().start()
