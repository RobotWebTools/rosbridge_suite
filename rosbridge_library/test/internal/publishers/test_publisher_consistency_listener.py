#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import rostest
import unittest

from time import sleep, time

from rosbridge_library.internal.publishers import *
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import *
from std_msgs.msg import String, Int32


class TestPublisherConsistencyListener(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_publisher_consistency_listener")

    def test_listener_timeout(self):
        """ See whether the listener can correctly time out """
        topic = "/test_listener_timeout"
        type = String

        publisher = rospy.Publisher(topic, type)

        listener = PublisherConsistencyListener()
        listener.attach(publisher)

        self.assertFalse(listener.timed_out())

        sleep(listener.timeout / 2.0)

        self.assertFalse(listener.timed_out())

        sleep(listener.timeout / 2.0 + 0.1)

        self.assertTrue(listener.timed_out())

    def test_listener_attach_detach(self):
        """ See whether the listener actually attaches and detaches itself """
        topic = "/test_listener_attach_detach"
        type = String

        publisher = rospy.Publisher(topic, type)
        orig_publish = publisher.publish

        listener = PublisherConsistencyListener()
        listener_publish = listener.publish_override

        self.assertNotEqual(orig_publish, listener_publish)
        self.assertNotIn(listener, publisher.impl.subscriber_listeners)

        listener.attach(publisher)

        self.assertEqual(publisher.publish, listener_publish)
        self.assertNotEqual(publisher.publish, orig_publish)
        self.assertIn(listener, publisher.impl.subscriber_listeners)

        listener.detach()

        self.assertEqual(publisher.publish, orig_publish)
        self.assertNotEqual(publisher.publish, listener_publish)
        self.assertNotIn(listener, publisher.impl.subscriber_listeners)

    def test_immediate_publish_fails_without(self):
        """ This test makes sure the failure case that the PublisherConsistency
        Listener is trying to solve, is indeed a failure case """
        topic = "/test_immediate_publish_fails_without"
        msg_class = String

        msg = String()
        string = "why halo thar"
        msg.data = string

        received = {"msg": None}
        def callback(msg):
            received["msg"] = msg

        rospy.Subscriber(topic, msg_class, callback)
        publisher = rospy.Publisher(topic, msg_class)
        publisher.publish(msg)
        sleep(0.5)

        self.assertNotEqual(received["msg"], msg)
        self.assertEqual(received["msg"], None)

    def test_immediate_publish(self):
        """ This test makes sure the PublisherConsistencyListener is working"""
        topic = "/test_immediate_publish"
        msg_class = String

        msg = String()
        string = "why halo thar"
        msg.data = string

        received = {"msg": None}
        def callback(msg):
            print("Received a msg! ", msg)
            received["msg"] = msg

        rospy.Subscriber(topic, msg_class, callback)

        class temp_listener(rospy.SubscribeListener):
            def peer_subscribe(self, topic_name, topic_publish, peer_publish):
                print("peer subscribe in temp listener")

        listener = PublisherConsistencyListener()
        publisher = rospy.Publisher(topic, msg_class, temp_listener())
        listener.attach(publisher)
        publisher.publish(msg)
        sleep(0.5)

        self.assertEqual(received["msg"], msg)

    def test_immediate_multi_publish_fails_without(self):
        """ This test makes sure the failure case that the PublisherConsistency
        Listener is trying to solve, is indeed a failure case, even for large
        message buffers """
        topic = "/test_immediate_multi_publish_fails_without"
        msg_class = Int32

        msgs = []
        for i in range(100):
            msg = Int32()
            msg.data = i
            msgs.append(msg)

        received = {"msgs": []}
        def callback(msg):
            received["msgs"].append(msg)

        rospy.Subscriber(topic, msg_class, callback)

        publisher = rospy.Publisher(topic, msg_class)
        for msg in msgs:
            publisher.publish(msg)
        sleep(0.5)

        self.assertEqual(len(received["msgs"]), 0)
        self.assertNotEqual(received["msgs"], msgs)

    def test_immediate_multi_publish(self):
        """ This test makes sure the PublisherConsistencyListener is working
        even with a huge message buffer"""
        topic = "/test_immediate_multi_publish"
        msg_class = Int32

        msgs = []
        for i in range(100):
            msg = Int32()
            msg.data = i
            msgs.append(msg)

        received = {"msgs": []}
        def callback(msg):
            received["msgs"].append(msg)

        rospy.Subscriber(topic, msg_class, callback)

        listener = PublisherConsistencyListener()
        publisher = rospy.Publisher(topic, msg_class)
        listener.attach(publisher)
        for msg in msgs:
            publisher.publish(msg)
        sleep(0.5)

        self.assertEqual(len(received["msgs"]), len(msgs))
        self.assertEqual(received["msgs"], msgs)


PKG = 'rosbridge_library'
NAME = 'test_publisher_consistency_listener'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestPublisherConsistencyListener)

