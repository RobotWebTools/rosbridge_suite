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

from rosbridge_library.capability import Capability
from rosbridge_library.internal import message_conversion, ros_loader
from rosbridge_library.protocol import Protocol


class ActionResult(Capability):

    action_result_msg_fields = [
        (True, "action", str),
        (False, "id", str),
        (False, "values", dict),
        (True, "status", int),
        (True, "result", bool),
    ]

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("action_result", self.action_result)

    def action_result(self, message: dict) -> None:
        # Typecheck the args
        self.basic_type_check(message, self.action_result_msg_fields)

        # check for the action
        action_name = message["action"]
        if action_name in self.protocol.external_action_list:
            action_handler = self.protocol.external_action_list[action_name]
            goal_id = message["id"]
            if message["result"]:
                # parse the message
                values = message["values"]
                status = message["status"]
                # create a message instance
                result = ros_loader.get_action_result_instance(action_handler.action_type)
                message_conversion.populate_instance(values, result)
                # pass along the result and status
                action_handler.handle_result(goal_id, result, status)
            else:
                # Abort the goal
                action_handler.handle_abort(goal_id)
        else:
            self.protocol.log(
                "error",
                f"Action {action_name} has not been advertised via rosbridge.",
            )
