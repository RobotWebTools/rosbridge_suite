#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest

from time import sleep

from rosbridge_library.internal.publishers import *
from rosbridge_library.internal.topics import *
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException
from std_msgs.msg import String, Int32


class TestPublisherManager(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_publisher_manager")
        manager.unregister_timeout = 1.0

    def is_topic_published(self, topicname):
        return topicname in dict(rospy.get_published_topics()).keys()

    def test_register_publisher(self):
        """ Register a publisher on a clean topic with a good msg type """
        topic = "/test_register_publisher"
        msg_type = "std_msgs/String"
        client = "client_test_register_publisher"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        manager.register(client, topic, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))

        manager.unregister(client, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        sleep(manager.unregister_timeout*1.1)
        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        self.assertFalse(topic in manager.unregister_timers)

    def test_register_publisher_multiclient(self):
        topic = "/test_register_publisher_multiclient"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_publisher_1"
        client2 = "client_test_register_publisher_2"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        manager.register(client1, topic, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        manager.register(client2, topic, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        manager.unregister(client1, topic)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        manager.unregister(client2, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        sleep(manager.unregister_timeout*1.1)
        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        self.assertFalse(topic in manager.unregister_timers)

    def test_register_publisher_conflicting_types(self):
        topic = "/test_register_publisher_conflicting_types"
        msg_type = "std_msgs/String"
        msg_type_bad = "std_msgs/Int32"
        client = "client_test_register_publisher_conflicting_types"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        manager.register(client, topic, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))

        self.assertRaises(TypeConflictException, manager.register, "client2", topic, msg_type_bad)

    def test_register_multiple_publishers(self):
        topic1 = "/test_register_multiple_publishers1"
        topic2 = "/test_register_multiple_publishers2"
        msg_type = "std_msgs/String"
        client = "client_test_register_multiple_publishers"

        self.assertFalse(topic1 in manager._publishers)
        self.assertFalse(topic2 in manager._publishers)
        self.assertFalse(self.is_topic_published(topic1))
        self.assertFalse(self.is_topic_published(topic2))
        manager.register(client, topic1, msg_type)
        self.assertTrue(topic1 in manager._publishers)
        self.assertTrue(self.is_topic_published(topic1))
        self.assertFalse(topic2 in manager._publishers)
        self.assertFalse(self.is_topic_published(topic2))
        manager.register(client, topic2, msg_type)
        self.assertTrue(topic1 in manager._publishers)
        self.assertTrue(self.is_topic_published(topic1))
        self.assertTrue(topic2 in manager._publishers)
        self.assertTrue(self.is_topic_published(topic2))

        manager.unregister(client, topic1)
        self.assertTrue(self.is_topic_published(topic1))
        self.assertTrue(topic1 in manager.unregister_timers)
        self.assertTrue(topic2 in manager._publishers)
        self.assertTrue(self.is_topic_published(topic2))

        manager.unregister(client, topic2)
        self.assertTrue(topic2 in manager.unregister_timers)
        self.assertTrue(self.is_topic_published(topic2))
        sleep(manager.unregister_timeout*1.1)
        self.assertFalse(topic1 in manager._publishers)
        self.assertFalse(self.is_topic_published(topic1))
        self.assertFalse(topic2 in manager._publishers)
        self.assertFalse(self.is_topic_published(topic2))
        self.assertFalse(topic1 in manager.unregister_timers)
        self.assertFalse(topic2 in manager.unregister_timers)

    def test_register_no_msgtype(self):
        topic = "/test_register_no_msgtype"
        client = "client_test_register_no_msgtype"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        self.assertRaises(TopicNotEstablishedException, manager.register, client, topic)

    def test_register_infer_topictype(self):
        topic = "/test_register_infer_topictype"
        client = "client_test_register_infer_topictype"

        self.assertFalse(self.is_topic_published(topic))

        rospy.Publisher(topic, String)

        self.assertTrue(self.is_topic_published(topic))
        self.assertFalse(topic in manager._publishers)
        manager.register(client, topic)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))

        manager.unregister(client, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(self.is_topic_published(topic))

    def test_register_multiple_notopictype(self):
        topic = "/test_register_multiple_notopictype"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_multiple_notopictype_1"
        client2 = "client_test_register_multiple_notopictype_2"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(topic in manager.unregister_timers)
        self.assertFalse(self.is_topic_published(topic))
        manager.register(client1, topic, msg_type)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        manager.register(client2, topic)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(self.is_topic_published(topic))
        manager.unregister(client1, topic)
        self.assertTrue(topic in manager._publishers)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(self.is_topic_published(topic))
        manager.unregister(client2, topic)
        self.assertTrue(topic in manager.unregister_timers)
        self.assertTrue(topic in manager._publishers)
        sleep(manager.unregister_timeout*1.1)
        self.assertFalse(topic in manager._publishers)
        self.assertFalse(topic in manager.unregister_timers)
        self.assertFalse(self.is_topic_published(topic))

    def test_publish_not_registered(self):
        topic = "/test_publish_not_registered"
        msg = {"data": "test publish not registered"}
        client = "client_test_publish_not_registered"

        self.assertFalse(topic in manager._publishers)
        self.assertFalse(self.is_topic_published(topic))
        self.assertRaises(TopicNotEstablishedException, manager.publish, client, topic, msg)

    def test_publisher_manager_publish(self):
        """ Make sure that publishing works """
        topic = "/test_publisher_manager_publish"
        msg = {"data": "test publisher manager publish"}
        client = "client_test_publisher_manager_publish"

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        rospy.Subscriber(topic, String, cb)
        manager.publish(client, topic, msg)
        sleep(0.5)

        self.assertEqual(received["msg"].data, msg["data"])

    def test_publisher_manager_bad_publish(self):
        """ Make sure that bad publishing fails """
        topic = "/test_publisher_manager_bad_publish"
        client = "client_test_publisher_manager_bad_publish"
        msg_type = "std_msgs/String"
        msg = {"data": 3}

        manager.register(client, topic, msg_type)
        self.assertRaises(FieldTypeMismatchException, manager.publish, client, topic, msg)


PKG = 'rosbridge_library'
NAME = 'test_publisher_manager'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestPublisherManager)

