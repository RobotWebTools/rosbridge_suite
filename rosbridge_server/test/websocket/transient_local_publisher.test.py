from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
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


class TestTransientLocalPublisher(unittest.TestCase):
    @websocket_test
    async def test_transient_local_publisher(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        pub_a = node.create_publisher(String, "/a_topic", qos_profile=qos)

        await sleep(node, 1)

        pub_a.publish(String(data="hello"))

        await sleep(node, 1)

        executor = node.executor
        assert executor is not None

        for num in range(3):
            ws_client = await make_client()
            ws_client.sendJson(
                {
                    "op": "subscribe",
                    "topic": "/a_topic",
                    "type": "std_msgs/String",
                }
            )

            ws_completed_future, ws_client.message_handler = expect_messages(
                1, "WebSocket " + str(num), node.get_logger()
            )
            ws_completed_future.add_done_callback(lambda _: executor.wake())

            self.assertEqual(
                await ws_completed_future,
                [{"op": "publish", "topic": "/a_topic", "msg": {"data": "hello"}}],
            )

        node.destroy_publisher(pub_a)
