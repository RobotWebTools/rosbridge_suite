from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING, Any

from std_srvs.srv import SetBool
from twisted.python import log

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.client import Client
    from rclpy.node import Node
    from rclpy.task import Future


sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, websocket_test

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestAdvertiseService(unittest.TestCase):
    @websocket_test
    async def test_two_concurrent_calls(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        ws_client = await make_client()
        ws_client.sendJson(
            {
                "op": "advertise_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service",
            }
        )
        client: Client = node.create_client(
            SetBool,  # type: ignore[arg-type]
            "/test_service",
        )
        client.wait_for_service()

        requests_future: Future[list[dict[str, Any]]]
        requests_future, ws_client.message_handler = expect_messages(
            2, "WebSocket", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        requests_future.add_done_callback(lambda _: executor.wake())

        response1_future = client.call_async(SetBool.Request(data=True))
        response2_future = client.call_async(SetBool.Request(data=False))

        requests = await requests_future
        assert requests is not None and len(requests) == 2

        self.assertEqual(requests[0]["op"], "call_service")
        self.assertEqual(requests[0]["service"], "/test_service")
        self.assertEqual(requests[0]["args"], {"data": True})
        ws_client.sendJson(
            {
                "op": "service_response",
                "service": "/test_service",
                "values": {"success": True, "message": "Hello world 1"},
                "id": requests[0]["id"],
                "result": True,
            }
        )

        self.assertEqual(requests[1]["op"], "call_service")
        self.assertEqual(requests[1]["service"], "/test_service")
        self.assertEqual(requests[1]["args"], {"data": False})
        ws_client.sendJson(
            {
                "op": "service_response",
                "service": "/test_service",
                "values": {"success": True, "message": "Hello world 2"},
                "id": requests[1]["id"],
                "result": True,
            }
        )

        self.assertEqual(
            await response1_future, SetBool.Response(success=True, message="Hello world 1")
        )
        self.assertEqual(
            await response2_future, SetBool.Response(success=True, message="Hello world 2")
        )

        node.destroy_client(client)
