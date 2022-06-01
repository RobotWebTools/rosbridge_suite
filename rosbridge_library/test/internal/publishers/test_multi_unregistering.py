#!/usr/bin/env python

import os
import rospy
import rosunit
import unittest

from time import sleep

from rosbridge_library.internal.publishers import MultiPublisher
from rosbridge_library.internal import ros_loader


PKG = 'rosbridge_library'
NAME = 'test_multi_unregistering'


class TestMultiUnregistering(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME, log_level=rospy.DEBUG)

    def test_publish_once(self):
        """ Make sure that publishing works """
        topic = "/test_publish_once"
        msg_type = "std_msgs/String"
        msg = {"data": "why halo thar"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        rospy.Subscriber(topic, ros_loader.get_message_class(msg_type), cb)
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(1)  # Time to publish and receive

        self.assertEqual(received["msg"].data, msg["data"])

    def test_publish_twice(self):
        """ Make sure that publishing works """
        topic = "/test_publish_twice"
        msg_type = "std_msgs/String"
        msg = {"data": "why halo thar"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        rospy.Subscriber(topic, ros_loader.get_message_class(msg_type), cb)
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(1)  # Time to publish and receive

        self.assertEqual(received["msg"].data, msg["data"])

        p.unregister()
        sleep(5)  # Time to unregister

        received["msg"] = None  # Reset received
        self.assertIsNone(received["msg"])
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(1)   # Time to publish and receive

        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestMultiUnregistering)
