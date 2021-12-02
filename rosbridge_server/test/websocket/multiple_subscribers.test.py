import os
import sys
import unittest

from rclpy.node import Node
from std_msgs.msg import String
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, sleep, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestMultipleSubscribers(unittest.TestCase):
    @websocket_test
    async def test_multiple_subscribers(self, node: Node, make_client):
        ws_client1 = await make_client()
        ws_client1.sendJson(
            {
                "op": "subscribe",
                "topic": "/a_topic",
                "type": "std_msgs/String",
            }
        )

        ws_client2 = await make_client()
        ws_client2.sendJson(
            {
                "op": "subscribe",
                "topic": "/a_topic",
                "type": "std_msgs/String",
            }
        )

        ws1_completed_future, ws_client1.message_handler = expect_messages(
            1, "WebSocket 1", node.get_logger()
        )
        ws1_completed_future.add_done_callback(lambda _: node.executor.wake())
        ws2_completed_future, ws_client2.message_handler = expect_messages(
            1, "WebSocket 2", node.get_logger()
        )
        ws2_completed_future.add_done_callback(lambda _: node.executor.wake())

        pub_a = node.create_publisher(String, "/a_topic", 1)

        await sleep(node, 1)

        pub_a.publish(String(data="hello"))

        self.assertEqual(
            await ws1_completed_future,
            [{"op": "publish", "topic": "/a_topic", "msg": {"data": "hello"}}],
        )
        self.assertEqual(
            await ws2_completed_future,
            [{"op": "publish", "topic": "/a_topic", "msg": {"data": "hello"}}],
        )
        node.destroy_publisher(pub_a)
