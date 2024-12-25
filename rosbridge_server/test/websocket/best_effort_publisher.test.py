import os
import sys
import unittest

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from twisted.python import log

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, sleep, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestBestEffortPublisher(unittest.TestCase):
    @websocket_test
    async def test_best_effort_publisher(self, node: Node, make_client):
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
        assert node.executor is not None
        ws1_completed_future.add_done_callback(lambda _: node.executor.wake())

        self.assertEqual(
            await ws1_completed_future,
            [{"op": "publish", "topic": "/a_topic", "msg": {"data": "hello"}}],
        )

        node.destroy_publisher(pub_a)
