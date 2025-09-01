from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING, Any

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.action.client import ClientGoalHandle
    from rclpy.node import Node
    from rclpy.task import Future
    from rclpy.type_support import FeedbackMessage

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestActionFeedback(unittest.TestCase):
    goal_result_future: Future | None
    latest_feedback: FeedbackMessage | None

    def goal_response_callback(self, future: Future[ClientGoalHandle]) -> None:
        goal_handle = future.result()
        assert goal_handle is not None
        if not goal_handle.accepted:
            return
        self.goal_result_future = goal_handle.get_result_async()

    def feedback_callback(self, msg: FeedbackMessage) -> None:
        self.latest_feedback = msg

    @websocket_test
    async def test_feedback(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        ws_client = await make_client()
        ws_client.sendJson(
            {
                "op": "advertise_action",
                "action": "/test_fibonacci_action",
                "type": "example_interfaces/Fibonacci",
            }
        )
        client: ActionClient = ActionClient(
            node,
            Fibonacci,  # type: ignore[arg-type]
            "/test_fibonacci_action",
        )
        client.wait_for_server()

        requests_future: Future[list[dict[str, Any]]]
        requests_future, ws_client.message_handler = expect_messages(
            1, "WebSocket", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        requests_future.add_done_callback(lambda _: executor.wake())

        self.goal_result_future = None
        goal_future = client.send_goal_async(
            Fibonacci.Goal(order=5),
            feedback_callback=self.feedback_callback,
        )
        goal_future.add_done_callback(self.goal_response_callback)

        requests = await requests_future
        assert requests is not None and len(requests) == 1

        self.assertEqual(requests[0]["op"], "send_action_goal")
        self.assertEqual(requests[0]["action"], "/test_fibonacci_action")
        self.assertEqual(requests[0]["action_type"], "example_interfaces/Fibonacci")
        self.assertEqual(requests[0]["args"], {"order": 5})

        self.latest_feedback = None
        ws_client.sendJson(
            {
                "op": "action_feedback",
                "action": "/test_fibonacci_action",
                "values": {"sequence": [0, 1, 1, 2]},
                "id": requests[0]["id"],
            }
        )
        ws_client.sendJson(
            {
                "op": "action_result",
                "action": "/test_fibonacci_action",
                "values": {"sequence": [0, 1, 1, 2, 3, 5]},
                "status": GoalStatus.STATUS_SUCCEEDED,
                "id": requests[0]["id"],
                "result": True,
            }
        )

        assert self.goal_result_future is not None
        result = await self.goal_result_future
        self.assertIsNotNone(self.latest_feedback)
        self.assertEqual(self.latest_feedback.feedback, Fibonacci.Feedback(sequence=[0, 1, 1, 2]))
        self.assertEqual(result.result, Fibonacci.Result(sequence=[0, 1, 1, 2, 3, 5]))

        node.destroy_client(client)
