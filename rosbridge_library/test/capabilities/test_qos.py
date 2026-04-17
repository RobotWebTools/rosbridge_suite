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
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
)
from rosbridge_library.internal.qos_extraction import extract_qos_profile
from rosbridge_library.protocol import Protocol
from std_msgs.msg import String

Qos_compatible_pub = {
    "durability": "volatile",
    "depth": 2,
    "deadline": 2,
    "lifespan": {"secs": 1, "nsecs": 8888},
}
Qos_compatible_sub = {
    "durability": "volatile",
    "depth": 2,
    "deadline": 2,
    "lifespan": {"secs": 1, "nsecs": 0},
}
Qos_incompatible_pub = {
    "durability": "volatile",
    "depth": 200,
    "deadline": 5,
}
Qos_incompatible_sub = {
    "durability": "transient_local",
    "depth": 150,
    "deadline": 4,
}


class TestQoS(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_qos")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self) -> None:
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_invalid_arguments(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_invalid_qos_args"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(String, topic, cb, subscriber_qos)

        msg = {"op": "publish", "msg_type": String, "topic": topic, "qos": "abcd"}
        self.assertRaises(InvalidArgumentException, pub.publish, msg)

    def test_incompatible_qos(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_incompatible_qos"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(
            String, topic, cb, qos_profile=extract_qos_profile(Qos_incompatible_sub)
        )

        msg = {"op": "publish", "msg_type": String, "topic": topic, "qos": Qos_incompatible_pub}
        pub.publish(msg)

        time.sleep(0.1)
        self.assertIsNone(received["msg"])

    def test_backward_compatibility(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_backward_compatibility"
        msg = {"data": "test if old publish works"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(String, topic, cb, 100)

        pub_msg = loads(
            dumps(
                {
                    "op": "publish",
                    "topic": topic,
                    "msg": msg,
                    "queue_size": 50,
                },
            ),
        )
        pub.publish(pub_msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])

    def test_publish_qos_works(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_qos_works"
        msg = {"data": "test publish qos works"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(String, topic, cb, extract_qos_profile(Qos_compatible_sub))

        pub_msg = loads(
            dumps(
                {
                    "op": "publish",
                    "topic": topic,
                    "msg": msg,
                    "qos": Qos_compatible_pub,
                },
            ),
        )
        pub.publish(pub_msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == "__main__":
    unittest.main()
