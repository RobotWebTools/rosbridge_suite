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
from typing import Any

from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse, ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rosbridge_library.capability import Capability
from rosbridge_library.internal import message_conversion
from rosbridge_library.internal.ros_loader import get_action_class
from rosbridge_library.protocol import Protocol


class AdvertisedActionHandler:

    id_counter = 1

    def __init__(
        self, action_name: str, action_type: str, protocol: Protocol, sleep_time: float = 0.001
    ) -> None:
        self.goal_futures: dict[str, Future] = {}
        self.goal_handles: dict[str, Any] = {}
        self.goal_statuses: dict[str, GoalStatus] = {}

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
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),  # https://github.com/ros2/rclpy/issues/834#issuecomment-961331870
        )

    def next_id(self) -> int:
        id = self.id_counter
        self.id_counter += 1
        return id

    async def execute_callback(self, goal: Any) -> Any:
        """Action server goal callback function."""
        # generate a unique ID
        goal_id = f"action_goal:{self.action_name}:{self.next_id()}"

        def done_callback(fut: Future) -> None:
            if fut.cancelled():
                goal.abort()
                self.protocol.log("info", f"Aborted goal {goal_id}")
                # Send an empty result to avoid stack traces
                fut.set_result(get_action_class(self.action_type).Result())
            else:
                if goal_id not in self.goal_statuses:
                    goal.abort()
                    return

                status = self.goal_statuses[goal_id]
                if status == GoalStatus.STATUS_SUCCEEDED:
                    goal.succeed()
                elif status == GoalStatus.STATUS_CANCELED:
                    goal.canceled()
                else:
                    goal.abort()

        future: Future = Future()
        future.add_done_callback(done_callback)
        self.goal_handles[goal_id] = goal
        self.goal_futures[goal_id] = future

        # build a request to send to the external client
        goal_message = {
            "op": "send_action_goal",
            "id": goal_id,
            "action": self.action_name,
            "action_type": self.action_type,
            "args": message_conversion.extract_values(goal.request),
            "feedback": True,
        }
        self.protocol.send(goal_message)

        try:
            return await future
        finally:
            del self.goal_futures[goal_id]
            del self.goal_handles[goal_id]

    def cancel_callback(self, cancel_request: ServerGoalHandle) -> CancelResponse:
        """Action server cancel callback function."""
        for goal_id, goal_handle in self.goal_handles.items():
            if cancel_request.goal_id == goal_handle.goal_id:
                self.protocol.log("warning", f"Canceling action {goal_id}")
                cancel_message = {
                    "op": "cancel_action_goal",
                    "id": goal_id,
                    "action": self.action_name,
                }
                self.protocol.send(cancel_message)
        return CancelResponse.ACCEPT

    def handle_feedback(self, goal_id: str, feedback: Any) -> None:
        """
        Called by the ActionFeedback capability to handle action feedback from the external client.
        """
        if goal_id in self.goal_handles:
            self.goal_handles[goal_id].publish_feedback(feedback)
        else:
            self.protocol.log("warning", f"Received action feedback for unrecognized id: {goal_id}")

    def handle_result(self, goal_id: str, result: dict, status: int) -> None:
        """
        Called by the ActionResult capability to handle a successful action result from the external client.
        """
        if goal_id in self.goal_futures:
            self.goal_statuses[goal_id] = status
            self.goal_futures[goal_id].set_result(result)
        else:
            self.protocol.log("warning", f"Received action result for unrecognized id: {goal_id}")

    def handle_abort(self, goal_id: str) -> None:
        """
        Called by the ActionResult capability to handle aborting action result from the external client.
        """
        if goal_id in self.goal_futures:
            self.goal_futures[goal_id].cancel()
        else:
            self.protocol.log(
                "warning", f"Received action abort request for unrecognized id: {goal_id}"
            )

    def graceful_shutdown(self) -> None:
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

        # Uncommenting this, you may get a segfault.
        # See https://github.com/ros2/rclcpp/issues/2163#issuecomment-1850925883
        # self.action_server.destroy()


class AdvertiseAction(Capability):
    actions_glob = None

    advertise_action_msg_fields = [(True, "action", str), (True, "type", str)]

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("advertise_action", self.advertise_action)

    def advertise_action(self, message: dict) -> None:
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
