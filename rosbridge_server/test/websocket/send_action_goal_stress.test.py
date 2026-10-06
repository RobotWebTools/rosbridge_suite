from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, sleep, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from example_interfaces.action._fibonacci import Fibonacci_Result
    from rclpy.action.server import ServerGoalHandle
    from rclpy.node import Node

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestSendActionGoalStress(unittest.TestCase):
    """
    Stress tests for send_action_goal.

    Each send_action_goal op creates a short-lived ActionClient on the
    rosbridge server side.  When many goals are sent in rapid succession
    (especially from multiple WebSocket connections), the ActionClient
    create/destroy calls race with the executor's wait-set iteration,
    which can cause:

        RCLError: Failed to add 'rcl_action_client_t' to wait set:
            action client pointer is invalid
    """

    def cancel_callback(self, _: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal: ServerGoalHandle) -> Fibonacci_Result:
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal.request.order):
            if goal.is_cancel_requested:
                goal.canceled()
                return Fibonacci.Result()

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal.publish_feedback(feedback_msg)  # type: ignore[arg-type]
            time.sleep(0.5)

        goal.succeed()
        return Fibonacci.Result(sequence=feedback_msg.sequence)

    @websocket_test
    async def test_rapid_action_goals(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        """
        Stress test ActionClient create/destroy against the executor.

        Since race conditions are probabilitic and CPU dependent, we send
        an unreasonable number of goals in a short period of time to test for
        reliability.

        Phase 1: Single client sends 50 goals with 5ms pacing.
        Phase 2: 5 clients send 10 goals each interleaved with 100ms pacing.
        """
        action_server = ActionServer(
            node,
            Fibonacci,
            "/test_fibonacci_action",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,  # type: ignore[arg-type]
            callback_group=ReentrantCallbackGroup(),
        )

        executor = node.executor
        assert executor is not None

        def assert_all_succeeded(responses: list[dict]) -> None:
            for resp in responses:
                self.assertEqual(resp["op"], "action_result")
                self.assertEqual(resp["action"], "/test_fibonacci_action")
                self.assertEqual(resp["result"], True)
                self.assertEqual(resp["status"], GoalStatus.STATUS_SUCCEEDED)

        # --- Phase 1: single client, rapid fire ---
        ws_client = await make_client()

        num_goals = 50
        responses_future, ws_client.message_handler = expect_messages(
            num_goals, "WebSocket (phase 1)", node.get_logger()
        )
        responses_future.add_done_callback(lambda _: executor.wake())

        # Duration of sleep between goals may result in different errors
        for i in range(num_goals):
            ws_client.sendJson(
                {
                    "op": "send_action_goal",
                    "id": f"goal_{i}",
                    "action": "/test_fibonacci_action",
                    "action_type": "example_interfaces/Fibonacci",
                    "args": {"order": 1},
                    "feedback": False,
                }
            )
            await sleep(node, 0.001)

        responses = await responses_future
        assert responses is not None and len(responses) == num_goals
        assert_all_succeeded(responses)

        # # --- Phase 2: multiple clients, interleaved ---
        num_ws_clients = 5
        goals_per_client = 10
        total_goals = num_ws_clients * goals_per_client

        ws_clients: list[TestClientProtocol] = []
        for _ in range(num_ws_clients):
            ws_clients.append(await make_client())  # noqa: PERF401

        responses_future, handler = expect_messages(
            total_goals, "WebSocket (phase 2)", node.get_logger()
        )
        for c in ws_clients:
            c.message_handler = handler
        responses_future.add_done_callback(lambda _: executor.wake())

        for i in range(goals_per_client):
            for ci, c in enumerate(ws_clients):
                c.sendJson(
                    {
                        "op": "send_action_goal",
                        "id": f"c{ci}_goal_{i}",
                        "action": "/test_fibonacci_action",
                        "action_type": "example_interfaces/Fibonacci",
                        "args": {"order": 1},
                        "feedback": False,
                    }
                )
                # await sleep(node, 0.1)
            # await sleep(node, 0.1)

        responses = await responses_future
        assert responses is not None and len(responses) == total_goals
        assert_all_succeeded(responses)

        action_server.destroy()
