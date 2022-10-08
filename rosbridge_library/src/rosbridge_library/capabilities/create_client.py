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


class createActionClient(Capability):

    createclient_msg_fields = [
        (True, "action_name", str),
        (True, "action_type", str),
        (True, "feedback", bool),
        (False, "throttle_rate", int),
        (False, "fragment_size", int),
        (False, "queue_length", int),
        (False, "compression", str),
    ]
    destroyclient_msg_fields = [(True, "action_type", str)]

    actions_glob = None

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("createClient", self.createClient)
        protocol.register_operation("destroyClient", self.destroyClient)
        protocol.register_operation("cancelGoal", self.cancelGoal)

        self._actionclients = {}

    def createClient(self, msg):
        # Check the args
        self.basic_type_check(msg, self.createclient_msg_fields)

        # Make the subscription
        action_name = msg.get("action_name")
        action_type = msg.get("action_type")
        goal_msg = msg.get("goal_msg", [])
        feedback = msg.get("feedback", True)

        if createActionClient.actions_glob is not None and createActionClient.actions_glob:
            self.protocol.log("info", "Action security glob enabled, checking action: " + action_type)
            match = False
            for glob in createActionClient.actions_glob:
                if fnmatch.fnmatch(action_type, glob):
                    self.protocol.log(
                        "info",
                        "Found match with glob " + glob + ", creating Action client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action, cancelling creation of action client type: " + action_type,
                )
                return
        else:
            self.protocol.log("info", "No action security glob, not checking subscription.")

        if action_type not in self._actionclients:
            client_id = msg.get("id", None)
            s_cb = partial(self._success, client_id, action_type)
            e_cb = partial(self._failure, client_id, action_type)
            if feedback:
                f_cb = partial(self._feedback, client_id, action_type)
            else:
                f_cb = None
            self._actionclients[action_type] = ActionCaller(
                action_type, action_name, goal_msg, s_cb, e_cb, f_cb, self.protocol.node_handle
            )

        # Register the subscriber
        self._actionclients[action_type].goal_msg = goal_msg
        self._actionclients[action_type].start()
    
    def _success(self, cid, action_type, message):
        outgoing_message = {
            "op": "action_response",
            "response_type":'result',
            "type": action_type,
            "values": message,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid, action_type, exc):
        outgoing_message = {
            "op": "action_response",
            "response_type": "error",
            "type": action_type,
            "values": str(exc),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)
    
    def _feedback(self, cid, action_type, message):
        outgoing_message = {
            "op": "action_response",
            "response_type":'feedback',
            "type": action_type,
            "values": extract_values(message),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)


    def destroyClient(self, msg):

        self.basic_type_check(msg, self.destroyclient_msg_fields)

        action_type = msg.get("action_type")

        if createActionClient.actions_glob is not None and createActionClient.actions_glob:
            self.protocol.log("info", "Action security glob enabled, checking action client of type:: " + action_type)
            match = False
            for glob in createActionClient.actions_glob:
                if fnmatch.fnmatch(action_type, glob):
                    self.protocol.log(
                        "info",
                        "Found match with glob " + glob + ", killing client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action client, cancelling destruction of action client of type: " + action_type,
                )
                return
        else:
            self.protocol.log("info", "No action security glob, not checking Action Client.")

        if action_type not in self._actionclients:
            return
        self._actionclients[action_type].unregister()

        if self._actionclients.is_empty():
            self._actionclients.clear()

        self.protocol.log("info", "Destroyed Action Client of type %s" % action_type)

    def cancelGoal(self, msg):
        self.basic_type_check(msg, self.destroyclient_msg_fields)

        action_type = msg.get("action_type")

        if createActionClient.actions_glob is not None and createActionClient.actions_glob:
            self.protocol.log("info", "Action security glob enabled, checking action client of type:: " + action_type)
            match = False
            for glob in createActionClient.actions_glob:
                if fnmatch.fnmatch(action_type, glob):
                    self.protocol.log(
                        "info",
                        "Found match with glob " + glob + ", killing client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action client, cancelling destruction of action client of type: " + action_type,
                )
                return
        else:
            self.protocol.log("info", "No action security glob, not checking Action Client.")

        if action_type not in self._actionclients:
            self.protocol.log("info", "action client of type %s not available" % action_type)
            return
        self._actionclients[action_type].cancel_goal()

        if self._actionclients.is_empty():
            self._actionclients.clear()

        self.protocol.log("info", "cancelled goal %s" % action_type)

    def finish(self):
        for clients in self._actionclients.values():
            clients.unregister()
        self._actionclients.clear()
        self.protocol.unregister_operation("createClient")
        self.protocol.unregister_operation("destroyClient")




