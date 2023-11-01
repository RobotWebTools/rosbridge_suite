#!/usr/bin/env python
import os
import sys
import time
import unittest

from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestSendActionGoal(unittest.TestCase):
    def execute_callback(self, goal):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

    @websocket_test
    async def test_one_call(self, node: Node, make_client):
        action_server = ActionServer(
            node,
            Fibonacci,
            "/test_fibonacci_action",
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        ws_client = await make_client()
        responses_future, ws_client.message_handler = expect_messages(
            1, "WebSocket", node.get_logger()
        )
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
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0]["op"], "action_result")
        self.assertEqual(responses[0]["action"], "/test_fibonacci_action")
        self.assertEqual(responses[0]["values"]["result"]["sequence"], [0, 1, 1, 2, 3, 5])
        self.assertEqual(responses[0]["result"], True)

        action_server.destroy()
