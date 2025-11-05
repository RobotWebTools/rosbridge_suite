#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from threading import Thread
from typing import TYPE_CHECKING, Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException
from rosbridge_library.internal.publishers import MultiPublisher
from rosbridge_library.internal.topics import TypeConflictException
from rosbridge_library.util.ros import is_topic_published

if TYPE_CHECKING:
    from std_msgs.msg import String

    from rosbridge_library.internal.type_support import ROSMessage


class TestMultiPublisher(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.node = Node("test_multi_publisher")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self) -> None:
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_register_multipublisher(self) -> None:
        """Register a publisher on a clean topic with a good msg type."""
        topic = "/test_register_multipublisher"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_published(self.node, topic))
        MultiPublisher(topic, self.node, msg_type)
        self.assertTrue(is_topic_published(self.node, topic))

    def test_unregister_multipublisher(self) -> None:
        """Register and unregister a publisher on a clean topic with a good msg type."""
        topic = "/test_unregister_multipublisher"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_published(self.node, topic))
        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        self.assertTrue(is_topic_published(self.node, topic))
        p.unregister()
        self.assertFalse(is_topic_published(self.node, topic))

    def test_register_client(self) -> None:
        """Adds a publisher then removes it."""
        topic = "/test_register_client"
        msg_type = "std_msgs/String"
        client_id = "client1"

        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        self.assertFalse(p.has_clients())

        p.register_client(client_id)
        self.assertTrue(p.has_clients())

        p.unregister_client(client_id)
        self.assertFalse(p.has_clients())

    def test_register_multiple_clients(self) -> None:
        """Adds multiple publishers then removes them."""
        topic = "/test_register_multiple_clients"
        msg_type = "std_msgs/String"

        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        self.assertFalse(p.has_clients())

        for i in range(1000):
            p.register_client(f"client{i}")
            self.assertTrue(p.has_clients())

        for i in range(1000):
            self.assertTrue(p.has_clients())
            p.unregister_client(f"client{i}")

        self.assertFalse(p.has_clients())

    def test_verify_type(self) -> None:
        topic = "/test_verify_type"
        msg_type = "std_msgs/String"
        othertypes = [
            "geometry_msgs/Pose",
            "action_msgs/GoalStatus",
            "geometry_msgs/WrenchStamped",
            "stereo_msgs/DisparityImage",
            "nav_msgs/OccupancyGrid",
            "geometry_msgs/Point32",
            "trajectory_msgs/JointTrajectoryPoint",
            "diagnostic_msgs/KeyValue",
            "visualization_msgs/InteractiveMarkerUpdate",
            "nav_msgs/GridCells",
            "sensor_msgs/PointCloud2",
        ]

        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        p.verify_type(msg_type)
        for othertype in othertypes:
            self.assertRaises(TypeConflictException, p.verify_type, othertype)

    def test_publish(self) -> None:
        """Make sure that publishing works."""
        topic = "/test_publish"
        msg_type = "std_msgs/String"
        msg = {"data": "why hello there"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: ROSMessage) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            ros_loader.get_message_class(msg_type), topic, cb, subscriber_qos
        )

        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        p.publish(msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])

    def test_publish_twice(self) -> None:
        """Make sure that publishing works."""
        topic = "/test_publish_twice"
        msg_type = "std_msgs/String"
        msg = {"data": "why hello there"}

        received: dict[str, Any] = {"msg": None}

        def cb(msg: ROSMessage) -> None:
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            ros_loader.get_message_class(msg_type), topic, cb, subscriber_qos
        )

        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        p.publish(msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])

        p.unregister()
        # The publisher went away at time T. Here's the timeline of the events:
        # T+1 seconds - the subscriber will retry to reconnect
        # T+2 seconds - publish msg -> it's gone
        # T+3 seconds - publish msg -> OK
        time.sleep(1)

        received["msg"] = None
        self.assertIsNone(received["msg"])
        p = MultiPublisher(topic, self.node, msg_type)

        time.sleep(1)
        p.publish(msg)
        self.assertIsNone(received["msg"])

        time.sleep(1)
        p.publish(msg)
        self.assertEqual(received["msg"].data, msg["data"])

    def test_bad_publish(self) -> None:
        """Make sure that bad publishing fails."""
        topic = "/test_publish"
        msg_type = "std_msgs/String"
        msg = {"data": 3}

        p: MultiPublisher[String] = MultiPublisher(topic, self.node, msg_type)
        self.assertRaises(FieldTypeMismatchException, p.publish, msg)


if __name__ == "__main__":
    unittest.main()
