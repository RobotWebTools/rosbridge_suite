#!/usr/bin/env python
PKG = 'rosbridge_library'
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest("std_msgs")
import rospy

import unittest
import time


roslib.load_manifest("roscpp")
from roscpp.srv import GetLoggers

from json import loads, dumps
from std_msgs.msg import String

from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.protocol import Protocol
from rosbridge_library.protocol import InvalidArgumentException, MissingArgumentException


class TestCallService(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_call_service")

    def test_missing_arguments(self):
        proto = Protocol("test_missing_arguments")
        s = CallService(proto)
        msg = loads(dumps({"op": "call_service"}))
        self.assertRaises(MissingArgumentException, s._call_service, None, msg)

    def test_invalid_arguments(self):
        proto = Protocol("test_invalid_arguments")
        s = CallService(proto)

        msg = loads(dumps({"op": "call_service", "service": 3}))
        self.assertRaises(InvalidArgumentException, s._call_service, None, msg)

    def test_call_service_works(self):
        # First, call the service the 'proper' way
        p = rospy.ServiceProxy("/rosout/get_loggers", GetLoggers)
        ret = p()


        proto = Protocol("test_call_service_works")
        s = CallService(proto)
        msg = loads(dumps({"op": "call_service", "service": "/rosout/get_loggers"}))

        received = {"msg": None}

        def cb(msg, cid=None):
            received["msg"] = msg

        proto.send = cb

        s._call_service(None, msg)

        time.sleep(0.5)

        for x, y in zip(ret.loggers, received["msg"]["values"]["loggers"]):
            self.assertEqual(x.name, y["name"])
            self.assertEqual(x.level, y["level"])

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_call_service', TestCallService)
