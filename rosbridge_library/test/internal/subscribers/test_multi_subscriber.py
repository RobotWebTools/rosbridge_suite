#!/usr/bin/env python
import time
import unittest
from threading import Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.internal.subscribers import MultiSubscriber
from rosbridge_library.internal.topics import TypeConflictException
from rosbridge_library.util.ros import is_topic_subscribed
from std_msgs.msg import Int32, String


class TestMultiSubscriber(unittest.TestCase):
    def setUp(self):
        self.client_id = "test_client_id"

        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_multi_subscriber")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_register_multisubscriber(self):
        """Register a subscriber on a clean topic with a good msg type"""
        topic = "/test_register_multisubscriber"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_subscribed(self.node, topic))
        MultiSubscriber(topic, self.client_id, lambda *args: None, self.node, msg_type=msg_type)
        self.assertTrue(is_topic_subscribed(self.node, topic))

    def test_unregister_multisubscriber(self):
        """Register and unregister a subscriber on a clean topic with a good msg type"""
        topic = "/test_unregister_multisubscriber"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_subscribed(self.node, topic))
        multi = MultiSubscriber(
            topic, self.client_id, lambda *args: None, self.node, msg_type=msg_type
        )
        self.assertTrue(is_topic_subscribed(self.node, topic))
        multi.unregister()
        self.assertFalse(is_topic_subscribed(self.node, topic))

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

        s = MultiSubscriber(topic, self.client_id, lambda *args: None, self.node, msg_type=msg_type)
        s.verify_type(msg_type)
        for othertype in othertypes:
            self.assertRaises(TypeConflictException, s.verify_type, othertype)

    def test_subscribe_unsubscribe(self):
        topic = "/test_subscribe_unsubscribe"
        msg_type = "std_msgs/String"

        self.assertFalse(is_topic_subscribed(self.node, topic))
        multi = MultiSubscriber(
            topic, self.client_id, lambda *args: None, self.node, msg_type=msg_type
        )
        self.assertTrue(is_topic_subscribed(self.node, topic))
        self.assertEqual(len(multi.new_subscriptions), 0)

        multi.subscribe(self.client_id, None)
        self.assertEqual(len(multi.new_subscriptions), 1)

        multi.unsubscribe(self.client_id)
        self.assertEqual(len(multi.new_subscriptions), 0)

        multi.unregister()
        self.assertFalse(is_topic_subscribed(self.node, topic))

    def test_subscribe_receive_json(self):
        topic = "/test_subscribe_receive_json"
        msg_type = "std_msgs/String"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        publisher_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.node.create_publisher(String, topic, publisher_qos)
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg.get_json_values()

        MultiSubscriber(topic, self.client_id, cb, self.node, msg_type=msg_type)
        time.sleep(0.1)
        pub.publish(msg)
        time.sleep(0.1)
        self.assertEqual(msg.data, received["msg"]["data"])

    def test_subscribe_receive_json_multiple(self):
        topic = "/test_subscribe_receive_json_multiple"
        msg_type = "std_msgs/Int32"

        numbers = list(range(100))

        publisher_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.node.create_publisher(Int32, topic, publisher_qos)
        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg.get_json_values()["data"])

        MultiSubscriber(topic, self.client_id, cb, self.node, msg_type=msg_type)
        time.sleep(0.1)
        for x in numbers:
            msg = Int32()
            msg.data = x
            pub.publish(msg)
            time.sleep(0.01)
        time.sleep(0.1)
        self.assertEqual(numbers, received["msgs"])

    def test_unsubscribe_does_not_receive_further_msgs(self):
        topic = "/test_unsubscribe_does_not_receive_further_msgs"
        msg_type = "std_msgs/String"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        publisher_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.node.create_publisher(String, topic, publisher_qos)
        received = {"count": 0}

        def cb(msg):
            received["count"] = received["count"] + 1

        multi = MultiSubscriber(topic, self.client_id, cb, self.node, msg_type=msg_type)
        time.sleep(0.1)
        pub.publish(msg)
        time.sleep(0.1)
        self.assertEqual(received["count"], 1)
        multi.unsubscribe(self.client_id)
        time.sleep(0.1)
        pub.publish(msg)
        time.sleep(0.1)
        self.assertEqual(received["count"], 1)

    def test_multiple_subscribers(self):
        topic = "/test_subscribe_receive_json"
        msg_type = "std_msgs/String"
        client1 = "client_test_subscribe_receive_json_1"
        client2 = "client_test_subscribe_receive_json_2"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        publisher_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.node.create_publisher(String, topic, publisher_qos)

        received = {"msg1": None, "msg2": None}

        def cb1(msg):
            received["msg1"] = msg.get_json_values()

        def cb2(msg):
            received["msg2"] = msg.get_json_values()

        multi = MultiSubscriber(topic, client1, cb1, self.node, msg_type=msg_type)
        multi.subscribe(client2, cb2)
        time.sleep(0.1)
        pub.publish(msg)
        time.sleep(0.1)
        self.assertEqual(msg.data, received["msg1"]["data"])
        self.assertEqual(msg.data, received["msg2"]["data"])
