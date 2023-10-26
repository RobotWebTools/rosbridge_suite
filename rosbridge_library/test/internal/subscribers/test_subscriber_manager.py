#!/usr/bin/env python
import time
import unittest
from threading import Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.internal.subscribers import manager
from rosbridge_library.internal.topics import (
    TopicNotEstablishedException,
    TypeConflictException,
)
from rosbridge_library.util.ros import is_topic_subscribed
from std_msgs.msg import String


class TestSubscriberManager(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_subscriber_manager")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_subscribe(self):
        """Register a publisher on a clean topic with a good msg type"""
        topic = "/test_subscribe"
        msg_type = "std_msgs/String"
        client = "client_test_subscribe"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))
        manager.subscribe(client, topic, None, self.node, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.unsubscribe(client, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))

    def test_register_subscriber_multiclient(self):
        topic = "/test_register_subscriber_multiclient"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_subscriber_multiclient_1"
        client2 = "client_test_register_subscriber_multiclient_2"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))
        manager.subscribe(client1, topic, None, self.node, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.subscribe(client2, topic, None, self.node, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.unsubscribe(client1, topic)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.unsubscribe(client2, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))

    def test_register_publisher_conflicting_types(self):
        topic = "/test_register_publisher_conflicting_types"
        msg_type = "std_msgs/String"
        msg_type_bad = "std_msgs/Int32"
        client = "client_test_register_publisher_conflicting_types"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))
        manager.subscribe(client, topic, None, self.node, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        self.assertRaises(
            TypeConflictException,
            manager.subscribe,
            "client2",
            topic,
            None,
            self.node,
            msg_type_bad,
        )

    def test_register_multiple_publishers(self):
        topic1 = "/test_register_multiple_publishers1"
        topic2 = "/test_register_multiple_publishers2"
        msg_type = "std_msgs/String"
        client = "client_test_register_multiple_publishers"

        self.assertFalse(topic1 in manager._subscribers)
        self.assertFalse(topic2 in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic1))
        self.assertFalse(is_topic_subscribed(self.node, topic2))

        manager.subscribe(client, topic1, None, self.node, msg_type)
        self.assertTrue(topic1 in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic1))
        self.assertFalse(topic2 in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic2))

        manager.subscribe(client, topic2, None, self.node, msg_type)
        self.assertTrue(topic1 in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic1))
        self.assertTrue(topic2 in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic2))

        manager.unsubscribe(client, topic1)
        self.assertFalse(topic1 in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic1))
        self.assertTrue(topic2 in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic2))

        manager.unsubscribe(client, topic2)
        self.assertFalse(topic1 in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic1))
        self.assertFalse(topic2 in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic2))

    def test_register_no_msgtype(self):
        topic = "/test_register_no_msgtype"
        client = "client_test_register_no_msgtype"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))
        self.assertRaises(
            TopicNotEstablishedException, manager.subscribe, client, topic, None, self.node
        )

    def test_register_infer_topictype(self):
        topic = "/test_register_infer_topictype"
        client = "client_test_register_infer_topictype"

        self.assertFalse(is_topic_subscribed(self.node, topic))

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(String, topic, lambda *args: None, subscriber_qos)

        self.assertTrue(is_topic_subscribed(self.node, topic))
        self.assertFalse(topic in manager._subscribers)

        manager.subscribe(client, topic, None, self.node)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.unsubscribe(client, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

    def test_register_multiple_notopictype(self):
        topic = "/test_register_multiple_notopictype"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_multiple_notopictype_1"
        client2 = "client_test_register_multiple_notopictype_2"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))

        manager.subscribe(client1, topic, None, self.node, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.subscribe(client2, topic, None, self.node)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.unsubscribe(client1, topic)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(is_topic_subscribed(self.node, topic))

        manager.unsubscribe(client2, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))

    def test_subscribe_not_registered(self):
        topic = "/test_subscribe_not_registered"
        client = "client_test_subscribe_not_registered"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(is_topic_subscribed(self.node, topic))
        self.assertRaises(
            TopicNotEstablishedException, manager.subscribe, client, topic, None, self.node
        )

    def test_publisher_manager_publish(self):
        topic = "/test_publisher_manager_publish"
        msg_type = "std_msgs/String"
        client = "client_test_publisher_manager_publish"

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

        manager.subscribe(client, topic, cb, self.node, msg_type)
        time.sleep(0.1)
        pub.publish(msg)
        time.sleep(0.1)
        self.assertEqual(msg.data, received["msg"]["data"])
