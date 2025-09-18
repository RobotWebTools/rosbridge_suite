from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from std_msgs.msg import String
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import expect_messages, sleep, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.node import Node


log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestMultipleSubscribers(unittest.TestCase):
    @websocket_test
    async def test_multiple_subscribers(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        sub_operation_json = {
            "op": "subscribe",
            "topic": "/a_topic",
            "type": "std_msgs/String",
            "compression": "cbor-raw",
        }
        ws_client1 = await make_client()
        ws_client1.sendJson(sub_operation_json)
        ws_client2 = await make_client()
        ws_client2.sendJson(sub_operation_json)

        ws1_completed_future, ws_client1.message_handler = expect_messages(
            1, "WebSocket 1", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        ws1_completed_future.add_done_callback(lambda _: executor.wake())
        ws2_completed_future, ws_client2.message_handler = expect_messages(
            1, "WebSocket 2", node.get_logger()
        )
        ws2_completed_future.add_done_callback(lambda _: executor.wake())

        pub_a = node.create_publisher(String, "/a_topic", 1)

        await sleep(node, 1)
        pub_a.publish(String(data="hello"))
        await sleep(node, 1)

        self.assertTrue(await ws1_completed_future)
        self.assertTrue(await ws2_completed_future)
        node.destroy_publisher(pub_a)
