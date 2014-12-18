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

from rosauth.srv import Authentication

from signal import signal, SIGINT, SIG_DFL
from functools import partial

from rosbridge_tools import find_tornado
tornado = find_tornado()
IOLoop = tornado.ioloop.IOLoop
Application = tornado.web.Application
WebSocketHandler = tornado.websocket.WebSocketHandler

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import json

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0
# if authentication should be used
authenticate = False

class RosbridgeWebSocket(WebSocketHandler):

    def open(self):
        global client_id_seed, clients_connected, authenticate
        try:
            self.protocol = RosbridgeProtocol(client_id_seed)
            self.protocol.outgoing = self.send_message
            self.authenticated = False
            client_id_seed = client_id_seed + 1
            clients_connected = clients_connected + 1
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        rospy.loginfo("Client connected.  %d clients total.", clients_connected)
        if authenticate:
            rospy.loginfo("Awaiting proper authentication...")

    def on_message(self, message):
        global authenticate
        # check if we need to authenticate
        if authenticate and not self.authenticated:
            try:
                msg = json.loads(message)
                if msg['op'] == 'auth':
                    # check the authorization information
                    auth_srv = rospy.ServiceProxy('authenticate', Authentication)
                    resp = auth_srv(msg['mac'], msg['client'], msg['dest'], 
                                                  msg['rand'], rospy.Time(msg['t']), msg['level'], 
                                                  rospy.Time(msg['end']))
                    self.authenticated = resp.authenticated
                    if self.authenticated:
                        rospy.loginfo("Client %d has authenticated.", self.protocol.client_id)
                        return
                # if we are here, no valid authentication was given
                rospy.logwarn("Client %d did not authenticate. Closing connection.", 
                              self.protocol.client_id)
                self.close()
            except:
                # proper error will be handled in the protocol class
                self.protocol.incoming(message)
        else:
            # no authentication required
            self.protocol.incoming(message)

    def on_close(self):
        global clients_connected
        clients_connected = clients_connected - 1
        self.protocol.finish()
        rospy.loginfo("Client disconnected. %d clients total.", clients_connected)

    def send_message(self, message):
        IOLoop.instance().add_callback(partial(self.write_message, message))

    def check_origin(self, origin):
        return True

if __name__ == "__main__":
    rospy.init_node("rosbridge_websocket")
    signal(SIGINT, SIG_DFL)

    # SSL options
    certfile = rospy.get_param('~certfile', None)
    keyfile = rospy.get_param('~keyfile', None)
    # if authentication should be used
    authenticate = rospy.get_param('~authenticate', False)
    port = rospy.get_param('~port', 9090)
    address = rospy.get_param('~address', "")

    if "--port" in sys.argv:
        idx = sys.argv.index("--port")+1
        if idx < len(sys.argv):
            port = int(sys.argv[idx])
        else:
            print "--port argument provided without a value."
            sys.exit(-1)

    application = Application([(r"/", RosbridgeWebSocket), (r"", RosbridgeWebSocket)])
    if certfile is not None and keyfile is not None:
        application.listen(port, address, ssl_options={ "certfile": certfile, "keyfile": keyfile})
    else:
        application.listen(port, address)
    rospy.loginfo("Rosbridge WebSocket server started on port %d", port)

    IOLoop.instance().start()
