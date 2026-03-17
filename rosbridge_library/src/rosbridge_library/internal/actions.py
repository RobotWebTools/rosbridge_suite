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

import time
from threading import Thread
from typing import TYPE_CHECKING, Any, Generic, cast

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.expand_topic_name import expand_topic_name

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

    from action_msgs.srv import CancelGoal_Response
    from rclpy.action.client import ClientGoalHandle
    from rclpy.node import Node
    from rclpy.task import Future
    from rclpy.type_support import FeedbackMessage, GetResultServiceResponse

    from rosbridge_library.internal.type_support import ROSMessage


class InvalidActionException(Exception):
    def __init__(self, action_name: str) -> None:
        Exception.__init__(self, f"Action {action_name} does not exist")


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


class SendGoal(Generic[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT]):
    """Helper class to send action goals."""

    result: GetResultServiceResponse[ROSActionResultT] | Exception | None = None

    def __init__(self, server_timeout_time: float = 1.0) -> None:
        self.server_timeout_time = server_timeout_time
        self.goal_handle: (
            ClientGoalHandle[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT]
            | None
        ) = None

    def get_result_cb(self, future: Future[GetResultServiceResponse[ROSActionResultT]]) -> None:
        self.result = future.result()

    def goal_response_cb(
        self,
        future: Future[
            ClientGoalHandle[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT]
        ],
    ) -> None:
        self.goal_handle = future.result()
        assert self.goal_handle is not None
        if not self.goal_handle.accepted:
            msg = "Action goal was rejected"
            self.result = Exception(msg)
            return
        result_future: Future[GetResultServiceResponse[ROSActionResultT]] = (
            self.goal_handle.get_result_async()
        )
        result_future.add_done_callback(self.get_result_cb)

    def goal_cancel_cb(self, _: Future[CancelGoal_Response]) -> None:
        self.goal_canceled = True

    async def send_goal(
        self,
        node_handle: Node,
        action: str,
        action_type: str,
        args: list | dict[str, Any] | None = None,
        feedback_cb: Callable[[FeedbackMessage[ROSActionFeedbackT]], None] | None = None,
    ) -> dict[str, Any]:
        # Given the action name and type, fetch a request instance
        action_name = expand_topic_name(action, node_handle.get_name(), node_handle.get_namespace())
        action_class = get_action_class(action_type)
        inst = cast("ROSActionGoalT", get_action_goal_instance(action_type))

        # Populate the instance with the provided args
        args_to_action_goal_instance(inst, args)

        client = ActionClient[ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT](
            node_handle, action_class, action_name, callback_group=ReentrantCallbackGroup()
        )
        if not client.wait_for_server(timeout_sec=self.server_timeout_time):
            msg = "No action server available"
            raise InvalidActionException(msg)

        send_goal_future = client.send_goal_async(inst, feedback_callback=feedback_cb)  # type: ignore[arg-type]
        send_goal_future.add_done_callback(self.goal_response_cb)

        self.goal_handle = await send_goal_future
        if self.goal_handle is None or not self.goal_handle.accepted:
            msg = "Action goal was rejected"
            raise Exception(msg)

        result_future = self.goal_handle.get_result_async()
        result = await result_future
        if result is None:
            msg = "Failed to get action result"
            raise Exception(msg)

        client.destroy()

        # Turn the response into JSON and pass to the callback
        return extract_values(result)

    async def cancel_goal(self) -> None:
        if self.goal_handle is None:
            return

        cancel_goal_future = self.goal_handle.cancel_goal_async()
        await cancel_goal_future
