# Software License Agreement (BSD License)
#
# Copyright (c) 2023, PickNik Inc.
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
#  * Neither the name of the copyright holder nor the names of its
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
from threading import Thread
from typing import Any

from action_msgs.msg import GoalStatus
from rosbridge_library.capability import Capability
from rosbridge_library.internal.actions import ActionClientHandler
from rosbridge_library.internal.message_conversion import extract_values
from rosbridge_library.protocol import Protocol


class SendActionGoal(Capability):

    send_action_goal_msg_fields = [
        (True, "action", str),
        (True, "action_type", str),
        (False, "fragment_size", (int, type(None))),
        (False, "compression", str),
    ]
    cancel_action_goal_msg_fields = [(True, "action", str)]

    actions_glob = None
    client_handler_list: dict[str, ActionClientHandler] = {}

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        send_action_goals_in_new_thread = (
            protocol.node_handle.get_parameter("send_action_goals_in_new_thread")
            .get_parameter_value()
            .bool_value
        )
        if send_action_goals_in_new_thread:
            # Sends the action goal in a separate thread so multiple actions can be processed simultaneously.
            protocol.node_handle.get_logger().info("Sending action goals in new thread")
            protocol.register_operation(
                "send_action_goal",
                lambda msg: Thread(target=self.send_action_goal, args=(msg,)).start(),
            )
        else:
            # Sends the actions goal in this thread, so actions block and must be processed sequentially.
            protocol.node_handle.get_logger().info("Sending action goals in existing thread")
            protocol.register_operation("send_action_goal", self.send_action_goal)

        # Always register goal canceling in a new thread.
        protocol.register_operation(
            "cancel_action_goal",
            lambda msg: Thread(target=self.cancel_action_goal, args=(msg,)).start(),
        )

    def send_action_goal(self, message: dict) -> None:
        # Pull out the ID
        cid = message.get("id", None)

        # Typecheck the args
        self.basic_type_check(message, self.send_action_goal_msg_fields)

        # Extract the args
        action = message["action"]
        action_type = message["action_type"]
        fragment_size = message.get("fragment_size", None)
        compression = message.get("compression", "none")
        args = message.get("args", [])

        if SendActionGoal.actions_glob is not None and SendActionGoal.actions_glob:
            self.protocol.log("debug", f"Action security glob enabled, checking action: {action}")
            match = False
            for glob in SendActionGoal.actions_glob:
                if fnmatch.fnmatch(action, glob):
                    self.protocol.log(
                        "debug",
                        f"Found match with glob {glob}, continuing sending action goal...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    f"No match found for action, cancelling sending action goal for: {action}",
                )
                return
        else:
            self.protocol.log("debug", "No action security glob, not checking sending action goal.")

        # Check for deprecated action ID, eg. /rosbridge/topics#33
        cid = extract_id(action, cid)

        # Create the callbacks
        s_cb = partial(self._success, cid, action, fragment_size, compression)
        e_cb = partial(self._failure, cid, action)
        if message.get("feedback", False):
            f_cb = partial(self._feedback, cid, action)
        else:
            f_cb = None

        # Run action client handler in the same thread.
        client_handler = ActionClientHandler(
            trim_action_name(action), action_type, args, s_cb, e_cb, f_cb, self.protocol.node_handle
        )
        self.client_handler_list[cid] = client_handler
        client_handler.run()
        del self.client_handler_list[cid]

    def cancel_action_goal(self, message: dict) -> None:
        # Extract the args
        cid = message.get("id", None)
        action = message["action"]

        # Typecheck the args
        self.basic_type_check(message, self.cancel_action_goal_msg_fields)

        # Pull out the ID
        # Check for deprecated action ID, eg. /rosbridge/topics#33
        cid = extract_id(action, cid)

        # Cancel the action
        if cid in self.client_handler_list:
            client_handler = self.client_handler_list[cid]
            if client_handler.send_goal_helper is not None:
                client_handler.send_goal_helper.cancel_goal()

    def _success(
        self, cid: str, action: str, fragment_size: int, compression: bool, message: dict
    ) -> None:
        outgoing_message = {
            "op": "action_result",
            "action": action,
            "values": message["result"],
            "status": message["status"],
            "result": True,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid: str, action: str, exc: Exception) -> None:
        self.protocol.log("error", f"send_action_goal {type(exc).__name__}: {cid}")
        # send response with result: false
        outgoing_message = {
            "op": "action_result",
            "action": action,
            "values": str(exc),
            "status": GoalStatus.STATUS_UNKNOWN,
            "result": False,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        self.protocol.send(outgoing_message)

    def _feedback(self, cid: str, action: str, message: Any) -> None:
        outgoing_message = {
            "op": "action_feedback",
            "action": action,
            "values": extract_values(message.feedback),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)


def trim_action_name(action: str) -> str:
    if "#" in action:
        return action[: action.find("#")]
    return action


def extract_id(action: str, cid: str) -> str:
    if cid is not None:
        return cid
    elif "#" in action:
        return action[action.find("#") + 1 :]
