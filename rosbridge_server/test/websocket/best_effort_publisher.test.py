from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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


class TestBestEffortPublisher(unittest.TestCase):
    @websocket_test
    async def test_best_effort_publisher(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.SYSTEM_DEFAULT,
        )
        pub_a = node.create_publisher(String, "/a_topic", qos_profile=qos)

        await sleep(node, 1)  # wait for publisher to be set up

        ws_client1 = await make_client()
        ws_client1.sendJson(
            {
                "op": "subscribe",
                "topic": "/a_topic",
                "type": "std_msgs/String",
            }
        )

        await sleep(node, 1)  # wait for subscriber to be set up

        pub_a.publish(String(data="hello"))

        ws1_completed_future, ws_client1.message_handler = expect_messages(
            1, "WebSocket 1", node.get_logger()
        )
        executor = node.executor
        assert executor is not None
        ws1_completed_future.add_done_callback(lambda _: executor.wake())

        self.assertEqual(
            await ws1_completed_future,
            [{"op": "publish", "topic": "/a_topic", "msg": {"data": "hello"}}],
        )

        node.destroy_publisher(pub_a)
