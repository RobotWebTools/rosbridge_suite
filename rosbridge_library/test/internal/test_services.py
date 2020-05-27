#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import rostest
import unittest
import time
import random

from rosbridge_library.internal import services, ros_loader
from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException

from roscpp.srv import GetLoggers

if sys.version_info >= (3, 0):
    string_types = (str,)
else:
    string_types = (str, unicode)  # noqa: F821


def populate_random_args(d):
    # Given a dictionary d, replaces primitives with random values
    if isinstance(d, dict):
        for x in d:
            d[x] = populate_random_args(d[x])
        return d
    elif isinstance(d, str):
        return str(random.random())
    elif sys.version_info < (3,0) and isinstance(d, unicode):  # noqa: F821
        return unicode(random.random())  # noqa: F821
    elif isinstance(d, bool):
        return True
    elif isinstance(d, int):
        return random.randint(100, 200)
    elif isinstance(d, float):
        return 3.5
    else:
        return d


class ServiceTester:

    def __init__(self, name, srv_type):
        self.name = name
        self.srvClass = ros_loader.get_service_class(srv_type)
        self.service = rospy.Service(name, self.srvClass, self.callback)

    def start(self):
        req = self.srvClass._request_class()
        gen = c.extract_values(req)
        gen = populate_random_args(gen)
        self.input = gen
        services.ServiceCaller(self.name, gen, self.success, self.error).start()

    def callback(self, req):
        self.req = req
        time.sleep(0.5)
        rsp = self.srvClass._response_class()
        gen = c.extract_values(rsp)
        gen = populate_random_args(gen)
        try:
            rsp = c.populate_instance(gen, rsp)
        except:
            print("populating instance")
            print(rsp)
            print("populating with")
            print(gen)
            raise
        self.output = gen
        return rsp

    def success(self, rsp):
        self.rsp = rsp

    def error(self, exc):
        self.exc = exc

    def validate(self, equality_function):
        if hasattr(self, "exc"):
            print(self.exc)
            print(self.exc.message)
            raise self.exc
        equality_function(self.input, c.extract_values(self.req))
        equality_function(self.output, self.rsp)


class TestServices(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_services")

    def msgs_equal(self, msg1, msg2):
        if type(msg1) in string_types and type(msg2) in string_types:
            pass
        else:
            self.assertEqual(type(msg1), type(msg2))
        if type(msg1) in c.list_types:
            for x, y in zip(msg1, msg2):
                self.msgs_equal(x, y)
        elif type(msg1) in c.primitive_types or type(msg1) in c.string_types:
            self.assertEqual(msg1, msg2)
        else:
            for x in msg1:
                self.assertTrue(x in msg2)
            for x in msg2:
                self.assertTrue(x in msg1)
            for x in msg1:
                self.msgs_equal(msg1[x], msg2[x])

    def test_populate_request_args(self):
        # Test empty messages
        for srv_type in ["TestEmpty", "TestResponseOnly"]:
            cls = ros_loader.get_service_class("rosbridge_library/" + srv_type)
            for args in [[], {}, None]:
                # Should throw no exceptions
                services.args_to_service_request_instance("", cls._request_class(), args)

        # Test msgs with data message
        for srv_type in ["TestRequestOnly", "TestRequestAndResponse"]:
            cls = ros_loader.get_service_class("rosbridge_library/" + srv_type)
            for args in [[3], {"data": 3}]:
                # Should throw no exceptions
                services.args_to_service_request_instance("", cls._request_class(), args)
            self.assertRaises(FieldTypeMismatchException, services.args_to_service_request_instance, "", cls._request_class(), ["hello"])

        # Test message with multiple fields
        cls = ros_loader.get_service_class("rosbridge_library/TestMultipleRequestFields")
        for args in [[3, 3.5, "hello", False], {"int": 3, "float": 3.5, "string": "hello", "bool": False}]:
            # Should throw no exceptions
            services.args_to_service_request_instance("", cls._request_class(), args)

    def test_service_call(self):
        """ Test a simple getloggers service call """
        # First, call the service the 'proper' way
        p = rospy.ServiceProxy(rospy.get_name() + "/get_loggers", GetLoggers)
        p.wait_for_service(0.5)
        ret = p()

        # Now, call using the services
        json_ret = services.call_service(rospy.get_name() + "/get_loggers")
        for x, y in zip(ret.loggers, json_ret["loggers"]):
            self.assertEqual(x.name, y["name"])
            self.assertEqual(x.level, y["level"])

    def test_service_caller(self):
        """ Same as test_service_call but via the thread caller """
        # First, call the service the 'proper' way
        p = rospy.ServiceProxy(rospy.get_name() + "/get_loggers", GetLoggers)
        p.wait_for_service(0.5)
        ret = p()

        rcvd = {"json": None}

        def success(json):
            rcvd["json"] = json

        def error():
            raise Exception()

        # Now, call using the services
        services.ServiceCaller(rospy.get_name() + "/get_loggers", None, success, error).start()

        time.sleep(0.5)

        for x, y in zip(ret.loggers, rcvd["json"]["loggers"]):
            self.assertEqual(x.name, y["name"])
            self.assertEqual(x.level, y["level"])

    def test_service_tester(self):
        t = ServiceTester("/test_service_tester", "rosbridge_library/TestRequestAndResponse")
        t.start()
        time.sleep(1.0)
        t.validate(self.msgs_equal)

    def test_service_tester_alltypes(self):
        ts = []
        for srv in ["TestResponseOnly", "TestEmpty", "TestRequestAndResponse", "TestRequestOnly",
            "TestMultipleResponseFields", "TestMultipleRequestFields", "TestArrayRequest"]:
            t = ServiceTester("/test_service_tester_alltypes_" + srv, "rosbridge_library/" + srv)
            t.start()
            ts.append(t)

        time.sleep(1.0)

        for t in ts:
            t.validate(self.msgs_equal)

    def test_random_service_types(self):
        common = ["roscpp/GetLoggers", "roscpp/SetLoggerLevel",
        "std_srvs/Empty", "nav_msgs/GetMap", "nav_msgs/GetPlan",
        "sensor_msgs/SetCameraInfo", "topic_tools/MuxAdd",
        "topic_tools/MuxSelect", "tf2_msgs/FrameGraph",
        "rospy_tutorials/BadTwoInts", "rospy_tutorials/AddTwoInts"]
        ts = []
        for srv in common:
            t = ServiceTester("/test_random_service_types/" + srv, srv)
            t.start()
            ts.append(t)

        time.sleep(1.0)

        for t in ts:
            t.validate(self.msgs_equal)


PKG = 'rosbridge_library'
NAME = 'test_services'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestServices)

