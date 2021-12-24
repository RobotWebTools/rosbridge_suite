#!/usr/bin/env python
import sys
import rospy
import rosunit
import unittest
from time import sleep

from rosbridge_library.protocol import Protocol
from rosbridge_library.protocol import InvalidArgumentException, MissingArgumentException
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.internal.publishers import manager
from rosbridge_library.internal import ros_loader

from std_msgs.msg import String

from json import dumps, loads


PKG = 'rosbridge_library'
NAME = 'test_publish'


class TestPublish(unittest.TestCase):

    def setUp(self):
        rospy.init_node(NAME)

    def test_missing_arguments(self):
        proto = Protocol("hello")
        pub = Publish(proto)
        msg = {"op": "publish"}
        self.assertRaises(MissingArgumentException, pub.publish, msg)

        msg = {"op": "publish", "msg": {}}
        self.assertRaises(MissingArgumentException, pub.publish, msg)

    def test_invalid_arguments(self):
        proto = Protocol("hello")
        pub = Publish(proto)

        msg = {"op": "publish", "topic": 3}
        self.assertRaises(InvalidArgumentException, pub.publish, msg)

    def test_publish_works(self):
        proto = Protocol("hello")
        pub = Publish(proto)
        topic = "/test_publish_works"
        msg = {"data": "test publish works"}

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        rospy.Subscriber(topic, String, cb)

        pub_msg = loads(dumps({"op": "publish", "topic": topic, "msg": msg}))
        pub.publish(pub_msg)

        sleep(0.5)
        self.assertEqual(received["msg"].data, msg["data"])


if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestPublish)

