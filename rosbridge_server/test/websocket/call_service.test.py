from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.node import Node
    # from rclpy.task import Future

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestCallService(unittest.TestCase):
    @websocket_test
    async def test_one_call(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        def service_cb(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
            self.assertTrue(req.data)
            res.success = True
            res.message = "Hello, world!"
            return res

        service = node.create_service(
            SetBool,  # type: ignore[arg-type]
            "/test_service",
            service_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        ws_client = await make_client()
        responses_future, ws_client.message_handler = expect_messages(
            1, "WebSocket", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        responses_future.add_done_callback(lambda _: executor.wake())

        ws_client.sendJson(
            {
                "op": "call_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service",
                "args": {"data": True},
            }
        )

        responses = await responses_future
        assert responses is not None and len(responses) == 1

        self.assertEqual(responses[0]["op"], "service_response")
        self.assertEqual(responses[0]["service"], "/test_service")
        self.assertEqual(responses[0]["values"], {"success": True, "message": "Hello, world!"})
        self.assertEqual(responses[0]["result"], True)

        node.destroy_service(service)

        def service_long_cb(req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
            time.sleep(0.2)
            self.assertTrue(req.data)
            res.success = True
            res.message = "Hello, world!"
            return res

        service = node.create_service(
            SetBool,  # type: ignore[arg-type]
            "/test_service_long",
            service_long_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        responses_future, ws_client.message_handler = expect_messages(
            2, "WebSocket", node.get_logger()
        )
        responses_future.add_done_callback(lambda _: executor.wake())

        ws_client.sendJson(
            {
                "op": "call_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service_long",
                "args": {"data": True},
                "timeout": 0.1,
            }
        )

        ws_client.sendJson(
            {
                "op": "call_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service_long",
                "args": {"data": True},
                "timeout": 0.5,
            }
        )

        responses = await responses_future
        assert responses is not None and len(responses) == 2

        self.assertEqual(responses[0]["op"], "service_response")
        self.assertEqual(responses[0]["service"], "/test_service_long")
        self.assertEqual(
            responses[0]["values"], "Timeout exceeded while waiting for service response"
        )
        self.assertEqual(responses[0]["result"], False)

        self.assertEqual(responses[1]["op"], "service_response")
        self.assertEqual(responses[1]["service"], "/test_service_long")
        self.assertEqual(responses[1]["values"], {"success": True, "message": "Hello, world!"})
        self.assertEqual(responses[1]["result"], True)

        node.destroy_service(service)
