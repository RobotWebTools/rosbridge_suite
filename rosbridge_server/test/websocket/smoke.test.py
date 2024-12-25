#!/usr/bin/env python
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


class TestWebsocketSmoke(unittest.TestCase):
    @websocket_test
    async def test_smoke(self, node: Node, make_client):
        ws_client = await make_client()
        # For consistency, the number of messages must not exceed the the protocol
        # Subscriber queue_size.
        NUM_MSGS = 10
        MSG_SIZE = 10
        A_TOPIC = "/a_topic"
        B_TOPIC = "/b_topic"
        A_STRING = "A" * MSG_SIZE
        B_STRING = "B" * MSG_SIZE
        WARMUP_DELAY = 1.0  # seconds

        sub_completed_future, sub_handler = expect_messages(
            NUM_MSGS, "ROS subscriber", node.get_logger()
        )
        ws_completed_future, ws_client.message_handler = expect_messages(
            NUM_MSGS, "WebSocket", node.get_logger()
        )
        assert node.executor is not None
        ws_completed_future.add_done_callback(lambda _: node.executor.wake())

        sub_a = node.create_subscription(String, A_TOPIC, sub_handler, NUM_MSGS)
        pub_b = node.create_publisher(String, B_TOPIC, NUM_MSGS)

        ws_client.sendJson(
            {
                "op": "subscribe",
                "topic": B_TOPIC,
                "type": "std_msgs/String",
                "queue_length": 0,  # Test the roslib default.
            }
        )
        ws_client.sendJson(
            {
                "op": "advertise",
                "topic": A_TOPIC,
                "type": "std_msgs/String",
            }
        )

        await sleep(node, WARMUP_DELAY)

        ws_client.sendJson(
            {
                "op": "publish",
                "topic": A_TOPIC,
                "msg": {
                    "data": A_STRING,
                },
            },
            times=NUM_MSGS,
        )

        for _ in range(NUM_MSGS):
            pub_b.publish(String(data=B_STRING))

        ros_received = await sub_completed_future
        ws_received = await ws_completed_future

        for msg in ws_received:
            self.assertEqual("publish", msg["op"])
            self.assertEqual(B_TOPIC, msg["topic"])
            self.assertEqual(B_STRING, msg["msg"]["data"])
        self.assertEqual(NUM_MSGS, len(ws_received))

        for msg in ros_received:
            self.assertEqual(A_STRING, msg.data)
        self.assertEqual(NUM_MSGS, len(ros_received))

        node.destroy_subscription(sub_a)
        node.destroy_publisher(pub_b)
