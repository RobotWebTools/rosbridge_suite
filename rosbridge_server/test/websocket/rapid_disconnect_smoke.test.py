#!/usr/bin/env python
import asyncio
import os
import sys
import math
import unittest

import rclpy.task
from rclpy.node import Node
from std_msgs.msg import String
from twisted.python import log
from typing import Any, Awaitable, Callable

sys.path.append(os.path.dirname(__file__))  # enable importing from common.py in this directory

import common  # noqa: E402
from common import expect_messages, sleep, websocket_test  # noqa: E402

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description

class TestWebsocketSmoke(unittest.TestCase):
    def pub_sub_many(self, node: Node, duration: float, topic: str, sub_ws_clients, publisher, content) -> Awaitable[bool]:
        """
        Async-compatible function to publish and subscribe many times using a ROS timer.
        """
        future = rclpy.task.Future()
        self.count = 0

        node.get_logger().info('sub many')
        def sub_callback():
            node.get_logger().info('{}'.format(self.count))
            self.count = self.count + 1
            if self.count >= len(sub_ws_clients) * 2:
                future.set_result(True)
                timer.cancel()
                pub_timer.cancel()
                node.destroy_timer(timer)
                node.destroy_timer(pub_timer)
            elif self.count < len(sub_ws_clients):
                sub_ws_clients[self.count].sendJson(
                    {
                        "op": "subscribe",
                        "topic": topic,
                        "type": "std_msgs/String",
                        "queue_length": 0,  # Test the roslib default.
                    }
                )
            else:
                sub_ws_clients[math.floor(self.count/2)].dropConnection(True)

        def pub_callback():
            publisher.publish(String(data=content))

        pub_timer = node.create_timer(duration * 2, pub_callback)
        timer = node.create_timer(duration, sub_callback)
        return future


    @websocket_test
    async def test_smoke(self, node: Node, make_client):
        # Make a single publisher that pushes lots of data
        WARMUP_DELAY = 1.0  # seconds
        NUM_MSGS = 10
        MSG_SIZE = 10000
        A_TOPIC = "/a_topic"
        A_STRING = "A" * MSG_SIZE
        pub_a = node.create_publisher(String, A_TOPIC, NUM_MSGS)

        sub_ws_clients = []
        for _ in range(7):
            sub_ws_clients.append(await make_client())
        result = await self.pub_sub_many(node, 0.0001, A_TOPIC, sub_ws_clients, pub_a, A_STRING)
        self.assertTrue(result)

        node.destroy_publisher(pub_a)
