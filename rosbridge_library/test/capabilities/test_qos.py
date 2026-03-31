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
)
from rosbridge_library.internal.qos_extraction import extract_qos_profile
from rosbridge_library.protocol import Protocol

Qos_compatible_pub = {
    "durability": "volatile",
    "depth": 2,
    "deadline": [2],
    "lifespan": [1, 8888],
    "liveliness_lease_duration": "infinite",
}
Qos_compatible_sub = {
    "durability": "volatile",
    "depth": 2,
    "deadline": [2],
    "lifespan": [1, 0],
    "liveliness_lease_duration": "infinite",
}
Qos_incompatible_pub = {
    "durability": "volatile",
    "depth": 200,
    "deadline": [5],
    "liveliness": "automatic",
}
Qos_incompatible_sub = {
    "durability": "transient_local",
    "depth": 150,
    "deadline": [4],
    "liveliness": "manual_by_topic",
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

        pub_qos_obj = Qos_incompatible_pub
        _ = extract_qos_profile(pub_qos_obj)
        pub_qos: QoSProfile = _ if _ is not None else QoSProfile(depth=10)
        sub_qos_obj = Qos_incompatible_sub
        _ = extract_qos_profile(sub_qos_obj)
        sub_qos: QoSProfile = _ if _ is not None else QoSProfile(depth=10)

        self.assertIsNotNone(pub_qos)
        self.assertIsNotNone(sub_qos)

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(String, topic, cb, qos_profile=sub_qos)

        msg = {"op": "publish", "msg_type": String, "topic": topic, "qos": pub_qos_obj}
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
        pub_qos_obj = Qos_compatible_pub
        _ = extract_qos_profile(pub_qos_obj)
        pub_qos: QoSProfile = _ if _ is not None else QoSProfile(depth=10)
        sub_qos_obj = Qos_compatible_sub
        _ = extract_qos_profile(sub_qos_obj)
        sub_qos: QoSProfile = _ if _ is not None else QoSProfile(depth=10)

        self.assertIsNotNone(pub_qos)
        self.assertIsNotNone(sub_qos)

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(String, topic, cb, sub_qos)

        pub_msg = loads(
            dumps(
                {
                    "op": "publish",
                    "topic": topic,
                    "msg": msg,
                    "qos": pub_qos_obj,
                },
            ),
        )
        pub.publish(pub_msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == "__main__":
    unittest.main()
