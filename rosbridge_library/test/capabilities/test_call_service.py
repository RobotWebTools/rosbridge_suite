#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest
import time

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
        self.assertRaises(MissingArgumentException, s.call_service, msg)

    def test_invalid_arguments(self):
        proto = Protocol("test_invalid_arguments")
        s = CallService(proto)

        msg = loads(dumps({"op": "call_service", "service": 3}))
        self.assertRaises(InvalidArgumentException, s.call_service, msg)

    def test_call_service_works(self):
        # First, call the service the 'proper' way
        p = rospy.ServiceProxy("/rosout/get_loggers", GetLoggers)
        p.wait_for_service()
        time.sleep(1.0)
        ret = p()

        proto = Protocol("test_call_service_works")
        s = CallService(proto)
        msg = loads(dumps({"op": "call_service", "service": "/rosout/get_loggers"}))

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None):
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(msg)

        timeout = 5.0
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(timeout):
            if received["arrived"]:
                break
            time.sleep(0.1)

        self.assertTrue(received["msg"]["result"])
        for x, y in zip(ret.loggers, received["msg"]["values"]["loggers"]):
            self.assertEqual(x.name, y["name"])
            self.assertEqual(x.level, y["level"])

    def test_call_service_fail(self):
        proto = Protocol("test_call_service_fail")
        s = CallService(proto)
        send_msg = loads(dumps({"op": "call_service", "service": "/rosout/set_logger_level", "args": '["ros", "invalid"]'}))

        received = {"msg": None, "arrived": False}
        def cb(msg, cid=None):
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(send_msg)

        timeout = 5.0
        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(timeout):
            if received["arrived"]:
                break
            time.sleep(0.1)

        self.assertFalse(received["msg"]["result"])


PKG = 'rosbridge_library'
NAME = 'test_call_service'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestCallService)

