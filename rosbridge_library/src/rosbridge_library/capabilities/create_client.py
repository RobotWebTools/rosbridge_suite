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
from functools import partial
from threading import Lock

from rosbridge_library.capability import Capability
from rosbridge_library.internal.actions import ActionCaller
from rosbridge_library.internal.subscription_modifiers import MessageHandler
from rosbridge_library.internal.message_conversion import extract_values

try:
    from cbor import dumps as encode_cbor
except ImportError:
    from rosbridge_library.util.cbor import dumps as encode_cbor

try:
    from ujson import dumps as encode_json
except ImportError:
    try:
        from simplejson import dumps as encode_json
    except ImportError:
        from json import dumps as encode_json

from json import dumps, loads
import rclpy
from rosbridge_library.protocol import Protocol

class createActionClient(Capability):

    createclient_msg_fields = [
        (True, "action_name", str),
        (True, "action_type", str),
        (False, "throttle_rate", int),
        (False, "fragment_size", int),
        (False, "queue_length", int),
        (False, "compression", str),
    ]
    #destroyclient_msg_fields = [(True, "action_type", str)]

    actions_glob = None

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("createClient", self.createClient)
        #protocol.register_operation("destroyClient", self.destroyClient)

        self._actionclients = {}

    def createClient(self, msg):
        # Pull out the ID
        cid = msg.get("id", None)

        # Check the args
        self.basic_type_check(msg, self.createclient_msg_fields)

        # Make the subscription
        action_name = msg.get("action_name")
        action_type = msg.get("action_type")
        args = msg.get("args", [])

        if createActionClient.actions_glob is not None and createActionClient.actions_glob:
            self.protocol.log("info", "Action security glob enabled, checking action: " + action_type)
            match = False
            for glob in createActionClient.actions_glob:
                if fnmatch.fnmatch(action_type, glob):
                    self.protocol.log(
                        "info",
                        "Found match with glob " + glob + ", continuing subscription...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action, cancelling subscription to: " + action_type,
                )
                return
        else:
            self.protocol.log("info", "No action security glob, not checking subscription.")

        if action_type not in self._actionclients:
            client_id = msg.get("id", None)
            self.protocol.node_handle.get_logger().info(f" client id : = {client_id, cid} ")
            s_cb = partial(self._success, client_id, action_type)
            e_cb = partial(self._failure, client_id, action_type)
            f_cb = partial(self._feedback, client_id, action_type)
            self._actionclients[action_type] = ActionCaller(
                action_type, action_name, args, s_cb, e_cb, f_cb, self.protocol.node_handle
            )
            self.protocol.log("info", "created client")

        # Register the subscriber
        self._actionclients[action_type].args = args
        self._actionclients[action_type].start()
    
    def _success(self, cid, action_type, message):
        outgoing_message = {
            "op": "service_response",
            "type": action_type,
            "return_type":'result',
            "values": message,
            "result": True,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid, action_type, exc):
        outgoing_message = {
            "op": "service_response",
            "type": action_type,
            "values": str(exc),
            "result": False,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)
    
    def _feedback(self, cid, action_type, message):
        outgoing_message = {
            "op": "service_response",
            "type": action_type,
            "return_type":'feedback',
            "values": extract_values(message),
            "result": True,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def finish(self):
        #for clients in self._actionclients.values():
         #   clients.unregister()
        self._actionclients.clear()
        self.protocol.unregister_operation("createClient")




