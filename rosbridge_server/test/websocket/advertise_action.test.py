#!/usr/bin/env python
import os
import sys
import unittest

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestAdvertiseAction(unittest.TestCase):
    goal1_result_future: Future | None
    goal2_result_future: Future | None

    def goal1_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.goal1_result_future = goal_handle.get_result_async()

    def goal2_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.goal2_result_future = goal_handle.get_result_async()

    @websocket_test
    async def test_two_concurrent_calls(self, node: Node, make_client):
        ws_client = await make_client()
        ws_client.sendJson(
            {
                "op": "advertise_action",
                "action": "/test_fibonacci_action",
                "type": "example_interfaces/Fibonacci",
            }
        )
        client = ActionClient(node, Fibonacci, "/test_fibonacci_action")
        client.wait_for_server()

        requests_future, ws_client.message_handler = expect_messages(
            2, "WebSocket", node.get_logger()
        )
        assert node.executor is not None
        requests_future.add_done_callback(lambda _: node.executor.wake())

        self.goal1_result_future = None
        goal1_future = client.send_goal_async(Fibonacci.Goal(order=3))
        goal1_future.add_done_callback(self.goal1_response_callback)

        self.goal2_result_future = None
        goal2_future = client.send_goal_async(Fibonacci.Goal(order=5))
        goal2_future.add_done_callback(self.goal2_response_callback)

        requests = await requests_future

        self.assertEqual(requests[0]["op"], "send_action_goal")
        self.assertEqual(requests[0]["action"], "/test_fibonacci_action")
        self.assertEqual(requests[0]["action_type"], "example_interfaces/Fibonacci")
        self.assertEqual(requests[0]["args"], {"order": 3})
        ws_client.sendJson(
            {
                "op": "action_result",
                "action": "/test_fibonacci_action",
                "values": {"sequence": [0, 1, 1, 2]},
                "status": GoalStatus.STATUS_SUCCEEDED,
                "id": requests[0]["id"],
                "result": True,
            }
        )

        self.assertEqual(requests[1]["op"], "send_action_goal")
        self.assertEqual(requests[1]["action"], "/test_fibonacci_action")
        self.assertEqual(requests[1]["action_type"], "example_interfaces/Fibonacci")
        self.assertEqual(requests[1]["args"], {"order": 5})
        ws_client.sendJson(
            {
                "op": "action_result",
                "action": "/test_fibonacci_action",
                "values": {"sequence": [0, 1, 1, 2, 3, 5]},
                "status": GoalStatus.STATUS_SUCCEEDED,
                "id": requests[1]["id"],
                "result": True,
            }
        )

        assert self.goal1_result_future is not None
        result1 = await self.goal1_result_future
        self.assertEqual(result1.result, Fibonacci.Result(sequence=[0, 1, 1, 2]))
        assert self.goal2_result_future is not None
        result2 = await self.goal2_result_future
        self.assertEqual(result2.result, Fibonacci.Result(sequence=[0, 1, 1, 2, 3, 5]))

        node.destroy_client(client)
