#!/usr/bin/env python
import os
import sys
import unittest

from rclpy.node import Node
from std_srvs.srv import SetBool
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestAdvertiseService(unittest.TestCase):
    @websocket_test
    async def test_two_concurrent_calls(self, node: Node, make_client):
        ws_client = await make_client()
        ws_client.sendJson(
            {
                "op": "advertise_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service",
            }
        )
        client = node.create_client(SetBool, "/test_service")
        client.wait_for_service()

        requests_future, ws_client.message_handler = expect_messages(
            2, "WebSocket", node.get_logger()
        )
        assert node.executor is not None
        requests_future.add_done_callback(lambda _: node.executor.wake())

        response1_future = client.call_async(SetBool.Request(data=True))
        response2_future = client.call_async(SetBool.Request(data=False))

        requests = await requests_future
        self.assertEqual(len(requests), 2)

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
