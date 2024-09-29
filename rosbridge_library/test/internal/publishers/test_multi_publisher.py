#!/usr/bin/env python
import time
import unittest
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException
from rosbridge_library.internal.publishers import MultiPublisher
from rosbridge_library.internal.topics import TypeConflictException
from rosbridge_library.util.ros import is_topic_published


class TestMultiPublisher(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.node = Node("test_multi_publisher")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_register_multipublisher(self):
        """Register a publisher on a clean topic with a good msg type"""
        topic = "/test_register_multipublisher"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_published(self.node, topic))
        MultiPublisher(topic, self.node, msg_type)
        self.assertTrue(is_topic_published(self.node, topic))

    def test_unregister_multipublisher(self):
        """Register and unregister a publisher on a clean topic with a good msg type"""
        topic = "/test_unregister_multipublisher"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_published(self.node, topic))
        p = MultiPublisher(topic, self.node, msg_type)
        self.assertTrue(is_topic_published(self.node, topic))
        p.unregister()
        self.assertFalse(is_topic_published(self.node, topic))

    def test_register_client(self):
        """Adds a publisher then removes it."""
        topic = "/test_register_client"
        msg_type = "std_msgs/String"
        client_id = "client1"

        p = MultiPublisher(topic, self.node, msg_type)
        self.assertFalse(p.has_clients())

        p.register_client(client_id)
        self.assertTrue(p.has_clients())

        p.unregister_client(client_id)
        self.assertFalse(p.has_clients())

    def test_register_multiple_clients(self):
        """Adds multiple publishers then removes them."""
        topic = "/test_register_multiple_clients"
        msg_type = "std_msgs/String"

        p = MultiPublisher(topic, self.node, msg_type)
        self.assertFalse(p.has_clients())

        for i in range(1000):
            p.register_client(f"client{i}")
            self.assertTrue(p.has_clients())

        for i in range(1000):
            self.assertTrue(p.has_clients())
            p.unregister_client(f"client{i}")

        self.assertFalse(p.has_clients())

    def test_verify_type(self):
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

        p = MultiPublisher(topic, self.node, msg_type)
        p.verify_type(msg_type)
        for othertype in othertypes:
            self.assertRaises(TypeConflictException, p.verify_type, othertype)

    def test_publish(self):
        """Make sure that publishing works"""
        topic = "/test_publish"
        msg_type = "std_msgs/String"
        msg = {"data": "why hello there"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            ros_loader.get_message_class(msg_type), topic, cb, subscriber_qos
        )

        p = MultiPublisher(topic, self.node, msg_type)
        p.publish(msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"].data, msg["data"])

    def test_publish_twice(self):
        """Make sure that publishing works"""
        topic = "/test_publish_twice"
        msg_type = "std_msgs/String"
        msg = {"data": "why hello there"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            ros_loader.get_message_class(msg_type), topic, cb, subscriber_qos
        )

        p = MultiPublisher(topic, self.node, msg_type)
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

    def test_bad_publish(self):
        """Make sure that bad publishing fails"""
        topic = "/test_publish"
        msg_type = "std_msgs/String"
        msg = {"data": 3}

        p = MultiPublisher(topic, self.node, msg_type)
        self.assertRaises(FieldTypeMismatchException, p.publish, msg)
