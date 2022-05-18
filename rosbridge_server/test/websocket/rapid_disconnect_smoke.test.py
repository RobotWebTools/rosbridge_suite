#!/usr/bin/env python
import asyncio
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

    async def pub_many_times(publisher, content):
        for _ in range(1000):
            publisher.publish(String(data=content))
        return 0


    async def sub_many_times(topic, make_client):
        for _ in range(10):
            # Make several subscibers that will subscribe and then immediately disconnect
            sub_ws_clients = []
            for _ in range(100):
                ws_client = await make_client()
                ws_client.sendJson(
                    {
                        "op": "subscribe",
                        "topic": topic,
                        "type": "std_msgs/String",
                        "queue_length": 0,  # Test the roslib default.
                    }
                )
                sub_ws_clients.append(ws_client)
            for ws_client in sub_ws_clients:
                #client.sendClose()
                ws_client.dropConnection(True)


    @websocket_test
    async def test_smoke(self, node: Node, make_client):
        WARMUP_DELAY = 1.0  # seconds
        # Make a single publisher that pushes lots of data
        # pub_ws_client = await make_client()
        NUM_MSGS = 10
        MSG_SIZE = 10000
        A_TOPIC = "/a_topic"
        A_STRING = "A" * MSG_SIZE
        pub_a = node.create_publisher(String, A_TOPIC, NUM_MSGS)

        await sleep(node, WARMUP_DELAY)

        # https://python.plainenglish.io/how-to-avoid-issues-when-waiting-on-multiple-events-in-python-asyncio-48e22d148de7
        pub_task = asyncio.create_task(pub_many_times(pub_a, A_STRING), name="publishing")
        sub_task = asyncio.create_task(sub_many_times(A_TOPIC, make_client), name="subscribing")

        done, pending = await asyncio.wait([pub_task, sub_task], return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()

        node.destroy_publisher(pub_a)
