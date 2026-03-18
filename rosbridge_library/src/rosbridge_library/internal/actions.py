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
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Generic, cast

from rclpy.action import ActionClient

from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_action_class,
    get_action_goal_instance,
)
from rosbridge_library.internal.type_support import (
    ROSActionFeedbackT,
    ROSActionGoalT,
    ROSActionImplT,
    ROSActionResultT,
)

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.action.client import ClientGoalHandle
    from rclpy.node import Node
    from rclpy.type_support import FeedbackMessage

    from rosbridge_library.internal.type_support import ROSMessage


class InvalidActionException(Exception):
    def __init__(self, action_name: str) -> None:
        Exception.__init__(self, f"Action {action_name} does not exist")


class ActionClientHandler(
    Generic[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT]
):
    def __init__(
        self,
        action: str,
        action_type: str,
        args: list | dict[str, Any] | None,
        success_callback: Callable[[dict[str, Any]], None],
        error_callback: Callable[[Exception], None],
        feedback_callback: Callable[[FeedbackMessage[ROSActionFeedbackT]], None] | None,
        node_handle: Node,
        *,
        server_timeout_time: float = 1.0,
    ) -> None:
        """
        Create a client handler for the specified action.

        :param action: The name of the action to execute.
        :param action_type: The type of the action to execute.
        :param args: Arguments to pass to the action. Can be an ordered list, or a dict of
            name-value pairs. Anything else will be treated as though no arguments were provided
            (which is still valid for some kinds of actions)
        :param success_callback: A callback to call with the JSON result of the service call
        :param error_callback: A callback to call if an error occurs. The callback will be passed
            the exception that caused the failure
        :param node_handle: A ROS 2 node handle to call services
        :param server_timeout_time: Time to wait for the action server to become available
        """
        self.action = action
        self.action_type = action_type
        self.args = args
        self.success = success_callback
        self.error = error_callback
        self.feedback = feedback_callback
        self.node_handle = node_handle
        self.goal_handle: (
            ClientGoalHandle[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT]
            | None
        ) = None

        self.action_client = ActionClient[
            ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT
        ](node_handle, get_action_class(action_type), action)

        if not self.action_client.wait_for_server(timeout_sec=server_timeout_time):
            raise InvalidActionException(action)

    async def send_goal_and_wait_for_result(self) -> None:
        # Fetch a goal instance
        inst = cast("ROSActionGoalT", get_action_goal_instance(self.action_type))

        # Populate the instance with the provided args
        args_to_action_goal_instance(inst, self.args)

        # Send the goal and wait for the goal handle
        send_goal_future = self.action_client.send_goal_async(inst, feedback_callback=self.feedback)  # type: ignore[arg-type]

        self.goal_handle = await send_goal_future
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.error(Exception("Action goal was rejected"))
            return

        result_future = self.goal_handle.get_result_async()
        result = await result_future
        if result is None:
            self.error(Exception("Failed to get action result"))
            return

        values = extract_values(result)
        self.success(values)

    async def cancel_goal(self) -> None:
        if self.goal_handle is not None:
            cancel_goal_future = self.goal_handle.cancel_goal_async()
            _response = await cancel_goal_future
            # TODO: handle response

    def finish(self) -> None:
        self.action_client.destroy()


def args_to_action_goal_instance(inst: ROSMessage, args: list | dict[str, Any] | None) -> None:
    """
    Populate an action goal instance with the provided args.

    Propagates any exceptions that may be raised.

    :param args: Can be a dictionary of values, or a list, or None
    """
    msg = {}
    if isinstance(args, list):
        msg = dict(zip(inst.get_fields_and_field_types().keys(), args, strict=False))
    elif isinstance(args, dict):
        msg = args

    # Populate the provided instance, propagating any exceptions
    populate_instance(msg, inst)
