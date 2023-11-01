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

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rosbridge_library.capability import Capability
from rosbridge_library.internal import message_conversion
from rosbridge_library.internal.ros_loader import get_action_class


class AdvertisedActionHandler:

    id_counter = 1

    def __init__(self, action_name, action_type, protocol, sleep_time=0.001):
        self.goal_futures = {}
        self.goal_handles = {}

        self.action_name = action_name
        self.action_type = action_type
        self.protocol = protocol
        self.sleep_time = sleep_time
        # setup the action
        self.action_server = ActionServer(
            protocol.node_handle,
            get_action_class(action_type),
            action_name,
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),  # https://github.com/ros2/rclpy/issues/834#issuecomment-961331870
        )

    def next_id(self):
        id = self.id_counter
        self.id_counter += 1
        return id

    async def execute_callback(self, goal):
        # generate a unique ID
        goal_id = f"action_goal:{self.action_name}:{self.next_id()}"

        future = rclpy.task.Future()
        future.add_done_callback(lambda _: goal.succeed())
        self.goal_handles[goal_id] = goal
        self.goal_futures[goal_id] = future

        # build a request to send to the external client
        goal_message = {
            "op": "send_action_goal",
            "id": goal_id,
            "action": self.action_name,
            "action_type": self.action_type,
            "args": message_conversion.extract_values(goal.request),
        }
        self.protocol.send(goal_message)

        try:
            return await future
        finally:
            del self.goal_futures[goal_id]
            del self.goal_handles[goal_id]

    def handle_feedback(self, goal_id, feedback):
        """
        Called by the ActionFeedback capability to handle action feedback from the external client.
        """
        if goal_id in self.goal_handles:
            self.goal_handles[goal_id].publish_feedback(feedback)
        else:
            self.protocol.log("warning", f"Received action feedback for unrecognized id: {goal_id}")

    def handle_result(self, goal_id, res):
        """
        Called by the ActionResult capability to handle an action result from the external client.
        """
        if goal_id in self.goal_futures:
            self.goal_futures[goal_id].set_result(res)
        else:
            self.protocol.log("warning", f"Received action result for unrecognized id: {goal_id}")

    def graceful_shutdown(self):
        """
        Signal the AdvertisedActionHandler to shutdown.
        """
        if self.goal_futures:
            incomplete_ids = ", ".join(self.goal_futures.keys())
            self.protocol.log(
                "warning",
                f"Action {self.action_name} was unadvertised with an action in progress, "
                f"aborting action goals with request IDs {incomplete_ids}",
            )
            for future_id in self.goal_futures:
                future = self.goal_futures[future_id]
                future.set_exception(RuntimeError(f"Action {self.action_name} was unadvertised"))
        self.action_server.destroy()


class AdvertiseAction(Capability):
    actions_glob = None

    advertise_action_msg_fields = [(True, "action", str), (True, "type", str)]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("advertise_action", self.advertise_action)

    def advertise_action(self, message):
        # Typecheck the args
        self.basic_type_check(message, self.advertise_action_msg_fields)

        # parse the incoming message
        action_name = message["action"]

        if AdvertiseAction.actions_glob is not None and AdvertiseAction.actions_glob:
            self.protocol.log(
                "debug",
                "Action security glob enabled, checking action: " + action_name,
            )
            match = False
            for glob in AdvertiseAction.actions_glob:
                if fnmatch.fnmatch(action_name, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing action advertisement...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action, cancelling action advertisement for: "
                    + action_name,
                )
                return
        else:
            self.protocol.log(
                "debug", "No action security glob, not checking action advertisement."
            )

        # check for an existing entry
        if action_name in self.protocol.external_action_list.keys():
            self.protocol.log("warn", f"Duplicate action advertised. Overwriting {action_name}.")
            self.protocol.external_action_list[action_name].graceful_shutdown()
            del self.protocol.external_action_list[action_name]

        # setup and store the action information
        action_type = message["type"]
        action_handler = AdvertisedActionHandler(action_name, action_type, self.protocol)
        self.protocol.external_action_list[action_name] = action_handler
        self.protocol.log("info", f"Advertised action {action_name}")
