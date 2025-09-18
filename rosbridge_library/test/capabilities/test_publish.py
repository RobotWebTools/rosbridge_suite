#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from json import dumps, loads
from threading import Thread
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.protocol import Protocol


class TestAdvertise(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_publish")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self) -> None:
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_missing_arguments(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        msg: dict[str, Any] = {"op": "publish"}
        self.assertRaises(MissingArgumentException, pub.publish, msg)

        msg = {"op": "publish", "msg": {}}
        self.assertRaises(MissingArgumentException, pub.publish, msg)

    def test_invalid_arguments(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)

        msg = {"op": "publish", "topic": 3}
        self.assertRaises(InvalidArgumentException, pub.publish, msg)

    def test_publish_works(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_works"
        msg = {"data": "test publish works"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(String, topic, cb, subscriber_qos)

        pub_msg = loads(dumps({"op": "publish", "topic": topic, "msg": msg}))
        pub.publish(pub_msg)
        time.sleep(0.5)
        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == "__main__":
    unittest.main()
