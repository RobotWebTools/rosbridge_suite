#!/usr/bin/env python
import os
import sys
import time
import unittest

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestSendActionGoal(unittest.TestCase):
    def cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal.request.order):
            if goal.is_cancel_requested:
                goal.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal.succeed()
        return Fibonacci.Result(sequence=feedback_msg.sequence)

    @websocket_test
    async def test_one_call(self, node: Node, make_client):
        action_server = ActionServer(
            node,
            Fibonacci,
            "/test_fibonacci_action",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        ws_client = await make_client()
        responses_future, ws_client.message_handler = expect_messages(
            5, "WebSocket", node.get_logger()
        )
        assert node.executor is not None
        responses_future.add_done_callback(lambda _: node.executor.wake())

        ws_client.sendJson(
            {
                "op": "send_action_goal",
                "action": "/test_fibonacci_action",
                "action_type": "example_interfaces/Fibonacci",
                "args": {"order": 5},
                "feedback": True,
            }
        )

        responses = await responses_future
        expected_result = [0, 1, 1, 2, 3, 5]
        self.assertEqual(len(responses), 5)

        for idx in range(4):
            self.assertEqual(responses[idx]["op"], "action_feedback")
            self.assertEqual(responses[idx]["values"]["sequence"], expected_result[: idx + 3])

        self.assertEqual(responses[-1]["op"], "action_result")
        self.assertEqual(responses[-1]["action"], "/test_fibonacci_action")
        self.assertEqual(responses[-1]["values"]["sequence"], expected_result)
        self.assertEqual(responses[-1]["status"], GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(responses[-1]["result"], True)

        action_server.destroy()
