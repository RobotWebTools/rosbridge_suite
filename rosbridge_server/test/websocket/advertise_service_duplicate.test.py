from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from std_srvs.srv import SetBool
from twisted.python import log

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.client import Client
    from rclpy.node import Node


sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, sleep, websocket_test

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestAdvertiseService(unittest.TestCase):
    @websocket_test
    async def test_double_advertise(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        ws_client1 = await make_client()
        ws_client1.sendJson(
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

        requests1_future, ws_client1.message_handler = expect_messages(
            1, "WebSocket 1", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        requests1_future.add_done_callback(lambda _: executor.wake())

        client.call_async(SetBool.Request(data=True))

        requests1 = await requests1_future
        self.assertEqual(
            requests1,
            [
                {
                    "op": "call_service",
                    "service": "/test_service",
                    "id": "service_request:/test_service:1",
                    "args": {"data": True},
                }
            ],
        )

        ws_client1.sendClose()

        ws_client2 = await make_client()
        ws_client2.sendJson(
            {
                "op": "advertise_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service",
            }
        )

        # wait for the server to handle the new advertisement
        await sleep(node, 1)

        requests2_future, ws_client2.message_handler = expect_messages(
            1, "WebSocket 2", node.get_logger()
        )
        requests2_future.add_done_callback(lambda _: executor.wake())

        response2_future = client.call_async(SetBool.Request(data=False))

        requests2 = await requests2_future
        self.assertEqual(
            requests2,
            [
                {
                    "op": "call_service",
                    "id": "service_request:/test_service:1",
                    "service": "/test_service",
                    "args": {"data": False},
                }
            ],
        )

        ws_client2.sendJson(
            {
                "op": "service_response",
                "service": "/test_service",
                "values": {"success": True, "message": "Hello world 2"},
                "id": "service_request:/test_service:1",
                "result": True,
            }
        )

        self.assertEqual(
            await response2_future, SetBool.Response(success=True, message="Hello world 2")
        )
