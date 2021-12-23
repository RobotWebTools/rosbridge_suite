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

    @unittest.skipIf(os.environ.get("ROS_DISTRO", "") == "melodic", "Don't run new test on melodic")
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

    @unittest.skipUnless(os.environ.get("ROS_DISTRO", "") == "melodic", "Run old test only on melodic")
    def test_publish_twice_old(self):
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

        sleep(1)

        self.assertEqual(received["msg"].data, msg["data"])

        p.unregister()
        # The publisher went away at time T. Here's the timeline of the events:
        # T+1 seconds - the subscriber will retry to reconnect - fail
        # T+3 seconds - the subscriber will retry to reconnect - fail
        # T+5 seconds - publish msg -> it's gone
        # T+7 seconds - the subscriber will retry to reconnect - success
        # T+8 seconds - publish msg -> OK
        # T+11 seconds - we receive the message. Looks like a bug in reconnection...
        #                https://github.com/ros/ros_comm/blob/indigo-devel/clients/rospy/src/rospy/impl/tcpros_base.py#L733
        #                That line should probably be indented.
        sleep(5)

        received["msg"] = None
        self.assertIsNone(received["msg"])
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        self.assertIsNone(received["msg"])

        sleep(3)
        p.publish(msg)
        sleep(2)
        # Next two lines should be removed when this is fixed:
        # https://github.com/ros/ros_comm/blob/indigo-devel/clients/rospy/src/rospy/impl/tcpros_base.py#L733
        self.assertIsNone(received["msg"])
        sleep(3)
        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestMultiUnregistering)
