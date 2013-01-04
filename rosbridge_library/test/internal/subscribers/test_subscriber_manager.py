#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest
from rosgraph import Master

from time import sleep

from rosbridge_library.internal.subscribers import *
from rosbridge_library.internal.topics import *
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException
from std_msgs.msg import String, Int32


class TestSubscriberManager(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_subscriber_manager")

    def is_topic_published(self, topicname):
        return topicname in dict(rospy.get_published_topics()).keys()

    def is_topic_subscribed(self, topicname):
        return topicname in dict(Master("test_subscriber_manager").getSystemState()[1])

    def test_subscribe(self):
        """ Register a publisher on a clean topic with a good msg type """
        topic = "/test_subscribe"
        msg_type = "std_msgs/String"
        client = "client_test_subscribe"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))
        manager.subscribe(client, topic, None, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))

        manager.unsubscribe(client, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))

    def test_register_subscriber_multiclient(self):
        topic = "/test_register_subscriber_multiclient"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_subscriber_multiclient_1"
        client2 = "client_test_register_subscriber_multiclient_2"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))
        manager.subscribe(client1, topic, None, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))
        manager.subscribe(client2, topic, None, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))
        manager.unsubscribe(client1, topic)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))
        manager.unsubscribe(client2, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))

    def test_register_publisher_conflicting_types(self):
        topic = "/test_register_publisher_conflicting_types"
        msg_type = "std_msgs/String"
        msg_type_bad = "std_msgs/Int32"
        client = "client_test_register_publisher_conflicting_types"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))
        manager.subscribe(client, topic, None, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))

        self.assertRaises(TypeConflictException, manager.subscribe, "client2", topic, None, msg_type_bad)

    def test_register_multiple_publishers(self):
        topic1 = "/test_register_multiple_publishers1"
        topic2 = "/test_register_multiple_publishers2"
        msg_type = "std_msgs/String"
        client = "client_test_register_multiple_publishers"

        self.assertFalse(topic1 in manager._subscribers)
        self.assertFalse(topic2 in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic1))
        self.assertFalse(self.is_topic_subscribed(topic2))
        manager.subscribe(client, topic1, None, msg_type)
        self.assertTrue(topic1 in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic1))
        self.assertFalse(topic2 in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic2))
        manager.subscribe(client, topic2, None, msg_type)
        self.assertTrue(topic1 in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic1))
        self.assertTrue(topic2 in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic2))

        manager.unsubscribe(client, topic1)
        self.assertFalse(topic1 in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic1))
        self.assertTrue(topic2 in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic2))

        manager.unsubscribe(client, topic2)
        self.assertFalse(topic1 in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic1))
        self.assertFalse(topic2 in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic2))

    def test_register_no_msgtype(self):
        topic = "/test_register_no_msgtype"
        client = "client_test_register_no_msgtype"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))
        self.assertRaises(TopicNotEstablishedException, manager.subscribe, client, topic, None)

    def test_register_infer_topictype(self):
        topic = "/test_register_infer_topictype"
        client = "client_test_register_infer_topictype"

        self.assertFalse(self.is_topic_subscribed(topic))

        rospy.Subscriber(topic, String, None)

        self.assertTrue(self.is_topic_subscribed(topic))
        self.assertFalse(topic in manager._subscribers)
        manager.subscribe(client, topic, None)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))

        manager.unsubscribe(client, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))

    def test_register_multiple_notopictype(self):
        topic = "/test_register_multiple_notopictype"
        msg_type = "std_msgs/String"
        client1 = "client_test_register_multiple_notopictype_1"
        client2 = "client_test_register_multiple_notopictype_2"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))
        manager.subscribe(client1, topic, None, msg_type)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))
        manager.subscribe(client2, topic, None)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))
        manager.unsubscribe(client1, topic)
        self.assertTrue(topic in manager._subscribers)
        self.assertTrue(self.is_topic_subscribed(topic))
        manager.unsubscribe(client2, topic)
        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))

    def test_subscribe_not_registered(self):
        topic = "/test_subscribe_not_registered"
        client = "client_test_subscribe_not_registered"

        self.assertFalse(topic in manager._subscribers)
        self.assertFalse(self.is_topic_subscribed(topic))
        self.assertRaises(TopicNotEstablishedException, manager.subscribe, client, topic, None)

    def test_publisher_manager_publish(self):
        topic = "/test_publisher_manager_publish"
        msg_type = "std_msgs/String"
        client = "client_test_publisher_manager_publish"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        pub = rospy.Publisher(topic, String)
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        manager.subscribe(client, topic, cb, msg_type)
        sleep(0.5)
        pub.publish(msg)
        sleep(0.5)
        self.assertEqual(msg.data, received["msg"]["data"])


PKG = 'rosbridge_library'
NAME = 'test_subscriber_manager'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestSubscriberManager)

