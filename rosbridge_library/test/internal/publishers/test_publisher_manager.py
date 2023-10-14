#!/usr/bin/env python
import time
import unittest
from threading import Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException
from rosbridge_library.internal.publishers import manager
from rosbridge_library.internal.topics import (
    TopicNotEstablishedException,
    TypeConflictException,
)
from rosbridge_library.util.ros import is_topic_published
from std_msgs.msg import String

# Reduce this from its default of 10 to speed up tests
manager.unregister_timeout = 1.0


class TestPublisherManager(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_publisher_manager")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_register_publisher(self):
        """Register a publisher on a clean topic with a good msg type"""
        topic = "/test_register_publisher"
        msg_type = "std_msgs/String"
        client = "client_test_register_publisher"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))
        manager.register(client, topic, self.node, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.unregister(client, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        time.sleep(manager.unregister_timeout + 1.0)
        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))
        self.assertFalse(topic in manager.unregister_timers)

    def test_register_publisher_multiclient(self):
        topic = "/test_register_publisher_multiclient"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_publisher_1"
        client2 = "client_test_register_publisher_2"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))

        manager.register(client1, topic, self.node, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.register(client2, topic, self.node, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.unregister(client1, topic)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.unregister(client2, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        time.sleep(manager.unregister_timeout + 1.0)
        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))
        self.assertFalse(topic in manager.unregister_timers)

    def test_register_publisher_conflicting_types(self):
        topic = "/test_register_publisher_conflicting_types"
        msg_type = "std_msgs/String"
        msg_type_bad = "std_msgs/Int32"
        client = "client_test_register_publisher_conflicting_types"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))

        manager.register(client, topic, self.node, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        self.assertRaises(
            TypeConflictException, manager.register, "client2", topic, self.node, msg_type_bad
        )

    def test_register_multiple_publishers(self):
        topic1 = "/test_register_multiple_publishers1"
        topic2 = "/test_register_multiple_publishers2"
        msg_type = "std_msgs/String"
        client = "client_test_register_multiple_publishers"

        self.assertFalse(topic1 in manager._publishers)
        self.assertFalse(topic2 in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic1))
        self.assertFalse(is_topic_published(self.node, topic2))

        manager.register(client, topic1, self.node, msg_type)
        self.assertTrue(topic1 in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic1))
        self.assertFalse(topic2 in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic2))

        manager.register(client, topic2, self.node, msg_type)
        self.assertTrue(topic1 in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic1))
        self.assertTrue(topic2 in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic2))

        manager.unregister(client, topic1)
        self.assertTrue(is_topic_published(self.node, topic1))
        self.assertTrue(topic1 in manager.unregister_timers)
        self.assertTrue(topic2 in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic2))

        manager.unregister(client, topic2)
        self.assertTrue(topic2 in manager.unregister_timers)
        self.assertTrue(is_topic_published(self.node, topic2))

        time.sleep(manager.unregister_timeout + 1.0)
        self.assertFalse(topic1 in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic1))
        self.assertFalse(topic2 in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic2))
        self.assertFalse(topic1 in manager.unregister_timers)
        self.assertFalse(topic2 in manager.unregister_timers)

    def test_register_no_msgtype(self):
        topic = "/test_register_no_msgtype"
        client = "client_test_register_no_msgtype"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))
        self.assertRaises(TopicNotEstablishedException, manager.register, client, topic, self.node)

    def test_register_infer_topictype(self):
        topic = "/test_register_infer_topictype"
        client = "client_test_register_infer_topictype"

        self.assertFalse(is_topic_published(self.node, topic))

        publisher_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_publisher(String, topic, publisher_qos)
        time.sleep(0.1)

        self.assertTrue(is_topic_published(self.node, topic))
        self.assertFalse(topic in manager._publishers)

        manager.register(client, topic, self.node)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.unregister(client, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(is_topic_published(self.node, topic))

    def test_register_multiple_notopictype(self):
        topic = "/test_register_multiple_notopictype"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_multiple_notopictype_1"
        client2 = "client_test_register_multiple_notopictype_2"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(topic in manager.unregister_timers)
        self.assertFalse(is_topic_published(self.node, topic))

        manager.register(client1, topic, self.node, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.register(client2, topic, self.node)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.unregister(client1, topic)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(is_topic_published(self.node, topic))

        manager.unregister(client2, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(topic in manager._publishers)

        time.sleep(manager.unregister_timeout + 1.0)
        self.assertFalse(topic in manager._publishers)
        self.assertFalse(topic in manager.unregister_timers)
        self.assertFalse(is_topic_published(self.node, topic))

    def test_publish_not_registered(self):
        topic = "/test_publish_not_registered"
        msg = {"data": "test publish not registered"}
        client = "client_test_publish_not_registered"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(is_topic_published(self.node, topic))
        self.assertRaises(
            TopicNotEstablishedException, manager.publish, client, topic, msg, self.node
        )

    def test_publisher_manager_publish(self):
        """Make sure that publishing works"""
        topic = "/test_publisher_manager_publish"
        msg = {"data": "test publisher manager publish"}
        client = "client_test_publisher_manager_publish"

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        subscriber_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(String, topic, cb, subscriber_qos)

        manager.publish(client, topic, msg, self.node)
        time.sleep(0.5)
        self.assertEqual(received["msg"].data, msg["data"])

    def test_publisher_manager_bad_publish(self):
        """Make sure that bad publishing fails"""
        topic = "/test_publisher_manager_bad_publish"
        client = "client_test_publisher_manager_bad_publish"
        msg_type = "std_msgs/String"
        msg = {"data": 3}

        manager.register(client, topic, self.node, msg_type)
        self.assertRaises(
            FieldTypeMismatchException, manager.publish, client, topic, msg, self.node
        )
