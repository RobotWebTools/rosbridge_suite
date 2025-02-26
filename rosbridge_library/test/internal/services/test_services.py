#!/usr/bin/env python3
import random
import time
import unittest
from threading import Thread

import numpy as np
import rclpy
from rcl_interfaces.srv import ListParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal import ros_loader, services
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException


def populate_random_args(d):
    # Given a dictionary d, replaces primitives with random values
    if isinstance(d, dict):
        for x in d:
            d[x] = populate_random_args(d[x])
        return d
    elif isinstance(d, str):
        return str(random.random())
    elif isinstance(d, bool):
        return True
    elif isinstance(d, int):
        return random.randint(100, 200)
    elif isinstance(d, float):
        return 3.5
    else:
        return d


class ServiceTester:
    def __init__(self, executor, name, srv_type):
        self.name = name
        self.executor = executor
        self.node = Node("service_tester_" + srv_type.replace("/", "_"))
        self.executor.add_node(self.node)
        self.srvClass = ros_loader.get_service_class(srv_type)
        self.service = self.node.create_service(self.srvClass, name, self.callback)

    def __del__(self):
        self.executor.remove_node(self.node)

    def start(self):
        req = self.srvClass.Request()
        gen = c.extract_values(req)
        gen = populate_random_args(gen)
        self.input = gen
        thread = services.ServiceCaller(
            self.name,
            gen,
            5.0,
            self.success,
            self.error,
            self.node,
        )
        thread.start()
        thread.join()

    def callback(self, req, res):
        self.req = req
        time.sleep(0.1)
        gen = c.extract_values(res)
        gen = populate_random_args(gen)
        try:
            res = c.populate_instance(gen, res)
        except:  # noqa: E722  # Will print() and raise
            print("populating instance")
            print(res)
            print("populating with")
            print(gen)
            raise
        self.output = gen
        return res

    def success(self, rsp):
        self.rsp = rsp

    def error(self, exc):
        self.exc = exc

    def validate(self, equality_function):
        if hasattr(self, "exc"):
            print(self.exc)
            raise self.exc
        equality_function(self.input, c.extract_values(self.req))
        equality_function(self.output, self.rsp)


class TestServices(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_node")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        rclpy.shutdown()

    def msgs_equal(self, msg1, msg2):
        if isinstance(msg1, str) and isinstance(msg2, str):
            pass
        else:
            self.assertEqual(type(msg1), type(msg2))
        if type(msg1) in c.list_types:
            for x, y in zip(msg1, msg2):
                self.msgs_equal(x, y)
        elif type(msg1) in c.primitive_types or type(msg1) is str:
            self.assertEqual(msg1, msg2)
        elif np.issubdtype(type(msg1), np.number):
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
            cls = ros_loader.get_service_class("rosbridge_test_msgs/" + srv_type)
            for args in [[], {}, None]:
                # Should throw no exceptions
                services.args_to_service_request_instance("", cls.Request(), args)

        # Test msgs with data message
        for srv_type in ["TestRequestOnly", "TestRequestAndResponse"]:
            cls = ros_loader.get_service_class("rosbridge_test_msgs/" + srv_type)
            for args in [[3], {"data": 3}]:
                # Should throw no exceptions
                services.args_to_service_request_instance("", cls.Request(), args)
            self.assertRaises(
                FieldTypeMismatchException,
                services.args_to_service_request_instance,
                "",
                cls.Request(),
                ["hello"],
            )

        # Test message with multiple fields
        cls = ros_loader.get_service_class("rosbridge_test_msgs/TestMultipleRequestFields")
        for args in [
            [3, 3.5, "hello", False],
            {"int_value": 3, "float_value": 3.5, "string": "hello", "bool_value": False},
        ]:
            # Should throw no exceptions
            services.args_to_service_request_instance("", cls.Request(), args)

    def test_service_call(self):
        """Test a simple list_parameters service call"""
        # Prepare parameter
        self.node.declare_parameter("test_parameter", 1.0)

        # First, call the service the 'proper' way
        p = self.node.create_client(ListParameters, self.node.get_name() + "/list_parameters")
        p.wait_for_service(0.5)
        ret = p.call_async(ListParameters.Request())
        while not ret.done():
            time.sleep(0.1)
        self.node.destroy_client(p)

        # Now, call using the services
        json_ret = services.call_service(
            self.node,
            self.node.get_name() + "/list_parameters",
        )
        for x, y in zip(ret.result().result.names, json_ret["result"]["names"]):
            self.assertEqual(x, y)

    def test_service_caller(self):
        """Same as test_service_call but via the thread caller"""
        # Prepare parameter
        self.node.declare_parameter("test_parameter", 1.0)

        # First, call the service the 'proper' way
        p = self.node.create_client(ListParameters, self.node.get_name() + "/list_parameters")
        p.wait_for_service(0.5)
        ret = p.call_async(ListParameters.Request())
        while not ret.done():
            time.sleep(0.1)
        self.node.destroy_client(p)

        rcvd = {"json": None}

        def success(json):
            rcvd["json"] = json

        def error():
            raise Exception()

        # Now, call using the services
        services.ServiceCaller(
            self.node.get_name() + "/list_parameters",
            None,
            5.0,
            success,
            error,
            self.node,
        ).start()

        time.sleep(0.2)

        for x, y in zip(ret.result().result.names, rcvd["json"]["result"]["names"]):
            self.assertEqual(x, y)

    def test_service_tester(self):
        t = ServiceTester(
            self.executor, "/test_service_tester", "rosbridge_test_msgs/TestRequestAndResponse"
        )
        t.start()
        time.sleep(0.2)
        t.validate(self.msgs_equal)

    def test_service_tester_alltypes(self):
        ts = []
        for srv in [
            "TestResponseOnly",
            "TestEmpty",
            "TestRequestAndResponse",
            "TestRequestOnly",
            "TestMultipleResponseFields",
            "TestMultipleRequestFields",
            "TestArrayRequest",
        ]:
            t = ServiceTester(
                self.executor, "/test_service_tester_alltypes_" + srv, "rosbridge_test_msgs/" + srv
            )
            t.start()
            ts.append(t)

        time.sleep(0.2)

        for t in ts:
            t.validate(self.msgs_equal)

    def test_random_service_types(self):
        common = [
            "rcl_interfaces/ListParameters",
            "rcl_interfaces/SetParameters",
            "std_srvs/Empty",
            "nav_msgs/GetMap",
            "nav_msgs/GetPlan",
            "sensor_msgs/SetCameraInfo",
            "tf2_msgs/FrameGraph",
            "example_interfaces/AddTwoInts",
        ]
        ts = []
        for srv in common:
            t = ServiceTester(self.executor, "/test_random_service_types/" + srv, srv)
            t.start()
            ts.append(t)

        time.sleep(0.2)

        for t in ts:
            t.validate(self.msgs_equal)
