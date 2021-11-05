#!/usr/bin/env python
import unittest
from json import dumps, loads
from time import sleep

import rospy
import rostest
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.protocol import (
    InvalidArgumentException,
    MissingArgumentException,
    Protocol,
)
from std_msgs.msg import String


class TestAdvertise(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_advertise")

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


PKG = "rosbridge_library"
NAME = "test_publish"
if __name__ == "__main__":
    rostest.unitrun(PKG, NAME, TestAdvertise)
