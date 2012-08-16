#!/usr/bin/env python
PKG = 'rosbridge_library'
import roslib; roslib.load_manifest(PKG); roslib.load_manifest("std_msgs")
import rospy
from rospy import get_published_topics

from time import sleep, time

from rosbridge_library.internal.publishers import *
from rosbridge_library.internal.topics import *
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import *
from std_msgs.msg import String, Int32

import unittest

class TestMultiUnregistering(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_multi_unregistering")

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

        sleep(1)

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

        sleep(1)

        self.assertEqual(received["msg"].data, msg["data"])
        
        p.unregister()
        sleep(5)
        
        received["msg"] = None
        self.assertIsNone(received["msg"])
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(1)

        self.assertEqual(received["msg"].data, msg["data"])
        
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_multi_unregistering', TestMultiUnregistering)
