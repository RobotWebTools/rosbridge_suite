#!/usr/bin/env python
import os
import sys
import time
import unittest

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestCallService(unittest.TestCase):
    @websocket_test
    async def test_one_call(self, node: Node, make_client):
        def service_cb(req, res):
            self.assertTrue(req.data)
            res.success = True
            res.message = "Hello, world!"
            return res

        service = node.create_service(
            SetBool, "/test_service", service_cb, callback_group=ReentrantCallbackGroup()
        )

        ws_client = await make_client()
        responses_future, ws_client.message_handler = expect_messages(
            1, "WebSocket", node.get_logger()
        )
        responses_future.add_done_callback(lambda _: node.executor.wake())

        ws_client.sendJson(
            {
                "op": "call_service",
                "type": "std_srvs/SetBool",
                "service": "/test_service",
                "args": {"data": True},
            }
        )

        responses = await responses_future
        self.assertEqual(len(responses), 1)
        self.assertEqual(responses[0]["op"], "service_response")
        self.assertEqual(responses[0]["service"], "/test_service")
        self.assertEqual(responses[0]["values"], {"success": True, "message": "Hello, world!"})
        self.assertEqual(responses[0]["result"], True)

        node.destroy_service(service)

        def service_long_cb(req, res):
            time.sleep(0.2)
            self.assertTrue(req.data)
            res.success = True
            res.message = "Hello, world!"
            return res

        service = node.create_service(
            SetBool, "/test_service_long", service_long_cb, callback_group=ReentrantCallbackGroup()
        )

        responses_future, ws_client.message_handler = expect_messages(
            2, "WebSocket", node.get_logger()
        )
        responses_future.add_done_callback(lambda _: node.executor.wake())

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
        self.assertEqual(len(responses), 2)
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
