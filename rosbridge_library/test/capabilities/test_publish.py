#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from json import dumps, loads
from threading import Thread
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.protocol import Protocol
from sensor_msgs.msg import TimeReference
from std_msgs.msg import String


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

    def test_publish_header_stamp_auto_populated_when_stamp_omitted(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_header_stamp_auto_populated"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: PoseStamped) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(PoseStamped, topic, cb, subscriber_qos)

        # Publish with header present but stamp omitted
        pub_msg = {
            "op": "publish",
            "topic": topic,
            "type": "geometry_msgs/msg/PoseStamped",
            "msg": {"header": {}},
        }
        pub.publish(pub_msg)
        time.sleep(0.5)

        self.assertIsNotNone(received["msg"])
        stamp = received["msg"].header.stamp
        self.assertGreater(
            stamp.sec + stamp.nanosec,
            0,
            "Expected server to auto-populate header.stamp with current ROS time, but stamp is zero",
        )

    def test_publish_header_stamp_preserved_when_provided_by_client(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_header_stamp_preserved"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: PoseStamped) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(PoseStamped, topic, cb, subscriber_qos)

        expected_sec = 12345
        expected_nanosec = 67890
        pub_msg = {
            "op": "publish",
            "topic": topic,
            "type": "geometry_msgs/msg/PoseStamped",
            "msg": {
                "header": {
                    "stamp": {"sec": expected_sec, "nanosec": expected_nanosec},
                }
            },
        }
        pub.publish(pub_msg)
        time.sleep(0.5)

        self.assertIsNotNone(received["msg"])
        stamp = received["msg"].header.stamp
        self.assertEqual(stamp.sec, expected_sec)
        self.assertEqual(stamp.nanosec, expected_nanosec)

    def test_publish_header_stamp_auto_populated_when_header_omitted(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_header_stamp_auto_populated_no_header"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: PoseStamped) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(PoseStamped, topic, cb, subscriber_qos)

        # Publish with no header at all (the entire header field is omitted)
        pub_msg = {
            "op": "publish",
            "topic": topic,
            "type": "geometry_msgs/msg/PoseStamped",
            "msg": {},
        }
        pub.publish(pub_msg)
        time.sleep(0.5)

        self.assertIsNotNone(received["msg"])
        stamp = received["msg"].header.stamp
        self.assertGreater(
            stamp.sec + stamp.nanosec,
            0,
            "Expected server to auto-populate header.stamp even when header is entirely omitted",
        )

    def test_publish_time_field_now_string_populated(self) -> None:
        proto = Protocol("hello", self.node)
        pub = Publish(proto)
        topic = "/test_time_field_now"

        received: dict[str, Any] = {"msg": None}

        def cb(msg: TimeReference) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(TimeReference, topic, cb, subscriber_qos)

        pub_msg = {
            "op": "publish",
            "topic": topic,
            "type": "sensor_msgs/msg/TimeReference",
            "msg": {"time_ref": "now"},
        }
        pub.publish(pub_msg)
        time.sleep(0.5)

        self.assertIsNotNone(received["msg"])
        time_ref = received["msg"].time_ref
        self.assertGreater(
            time_ref.sec + time_ref.nanosec,
            0,
            'Expected server to populate time_ref with current ROS time when set to "now"',
        )


if __name__ == "__main__":
    unittest.main()
