"""Test that the server handles concurrent client requests without deadlocking."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING, Any

from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.node import Node
    from rclpy.task import Future

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description

NUM_CLIENTS = 5
REQUESTS_PER_CLIENT = 3


class TestStartupRace(unittest.TestCase):
    @websocket_test
    async def test_concurrent_clients_do_not_deadlock(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        total = NUM_CLIENTS * REQUESTS_PER_CLIENT

        responses_future: Future
        handler: Callable[[Any], None]
        responses_future, handler = expect_messages(
            total, "concurrent service calls", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        responses_future.add_done_callback(lambda _: executor.wake())

        for i in range(NUM_CLIENTS):
            ws_client = await make_client()
            ws_client.message_handler = handler
            for j in range(REQUESTS_PER_CLIENT):
                ws_client.sendJson(
                    {
                        "op": "call_service",
                        "service": "/rosbridge_websocket/get_parameters",
                        "args": {"names": ["port"]},
                        "id": f"client_{i}_req_{j}",
                    }
                )

        responses = await responses_future
        assert responses is not None
        self.assertEqual(len(responses), total)
        for resp in responses:
            self.assertEqual(resp["op"], "service_response")
            self.assertTrue(resp["result"])
