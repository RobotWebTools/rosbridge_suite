#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from threading import Event, Thread
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_system_default
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
)
from rosbridge_library.internal.publishers import manager
from rosbridge_library.internal.qos_extraction import extract_qos_profile
from rosbridge_library.protocol import Protocol
from std_msgs.msg import String

QOS_COMPATIBLE_PUB = {
    "durability": "volatile",
    "depth": 2,
    "deadline": 2,
    "lifespan": {"secs": 1, "nsecs": 8888},
}
QOS_COMPATIBLE_SUB = {
    "durability": "volatile",
    "depth": 2,
    "deadline": 2,
    "lifespan": {"secs": 1, "nsecs": 0},
}
QOS_INCOMPATIBLE_PUB = {
    "durability": "volatile",
    "depth": 200,
    "deadline": 5,
}
QOS_INCOMPATIBLE_SUB = {
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

    def test_empty_qos_dict_uses_system_default(self) -> None:
        self.assertEqual(extract_qos_profile({}), qos_profile_system_default)

    def test_invalid_arguments(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_invalid_qos_args"
        topic_type = "std_msgs/msg/String"

        invalid_qos_profiles: list[Any] = [
            # qos must be a dict
            "abcd",
            42,
            ["depth", 10],
            # invalid policy names
            {"depth": 10, "reliability": "fast"},
            {"depth": 10, "durability": "sticky"},
            {"history": "last_one", "depth": 10},
            # invalid depth
            {"depth": -1},
            {"depth": 1.5},
            {"depth": "ten"},
            # invalid duration strings
            {"depth": 10, "deadline": "soon"},
            {"depth": 10, "lifespan": "best_available"},  # only valid for deadline
            # invalid duration types
            {"depth": 10, "deadline": [1, 0]},
            {"depth": 10, "lifespan": False},
        ]

        for qos in invalid_qos_profiles:
            with self.subTest(qos=qos):
                msg = {
                    "op": "publish",
                    "topic": topic,
                    "type": topic_type,
                    "qos": qos,
                }
                self.node.get_logger().info(f"Testing invalid QoS profile: {qos}")
                self.assertRaises(InvalidArgumentException, pub.publish, msg)

    def test_incompatible_qos(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_incompatible_qos"
        topic_type = "std_msgs/msg/String"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(
            String, topic, cb, qos_profile=extract_qos_profile(QOS_INCOMPATIBLE_SUB)
        )

        msg = {"op": "publish", "topic": topic, "type": topic_type, "qos": QOS_INCOMPATIBLE_PUB}
        pub.publish(msg)

        time.sleep(0.1)
        self.assertIsNone(received["msg"])

    def test_backward_compatibility_queue_size(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_backward_compatibility_queue_size"
        topic_type = "std_msgs/msg/String"
        msg = {"data": "test queue_size"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(String, topic, cb, 100)

        pub.publish(
            {
                "op": "publish",
                "topic": topic,
                "type": topic_type,
                "msg": msg,
                "queue_size": 42,
            }
        )

        self.assertEqual(manager._publishers[topic].qos_profile.depth, 42)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])

    def test_backward_compatibility_latch(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_backward_compatibility_latch"
        topic_type = "std_msgs/msg/String"
        msg = {"data": "test latch"}

        pub.publish(
            {
                "op": "publish",
                "topic": topic,
                "type": topic_type,
                "msg": msg,
                "latch": True,
            }
        )

        qos_profile = manager._publishers[topic].qos_profile
        self.assertEqual(qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL)
        self.assertEqual(qos_profile.depth, 1)

        # Late-joining subscriber should receive the latched message. Block on
        # a threading.Event set in the callback instead of sleeping, so the
        # test does not flake when DDS discovery + late-joining delivery
        # overrun a short sleep on slower middleware setups.
        received: dict[str, Any] = {"msg": None}
        msg_received = Event()

        def cb(msg: String) -> None:
            received["msg"] = msg
            msg_received.set()

        self.node.create_subscription(String, topic, cb, qos_profile)
        self.assertTrue(
            msg_received.wait(timeout=5.0),
            "Late-joining subscriber did not receive latched message within 5 s",
        )
        self.assertEqual(received["msg"].data, msg["data"])

    def test_publish_qos_works(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_publish_qos_works"
        topic_type = "std_msgs/msg/String"
        msg = {"data": "test publish qos works"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: String) -> None:
            received["msg"] = msg

        self.node.create_subscription(String, topic, cb, extract_qos_profile(QOS_COMPATIBLE_SUB))

        pub.publish(
            {
                "op": "publish",
                "topic": topic,
                "type": topic_type,
                "msg": msg,
                "qos": QOS_COMPATIBLE_PUB,
            }
        )
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == "__main__":
    unittest.main()
