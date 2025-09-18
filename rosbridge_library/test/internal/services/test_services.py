#!/usr/bin/env python3
from __future__ import annotations

import random
import time
import unittest
from threading import Thread
from typing import TYPE_CHECKING, Any, NoReturn

import numpy as np
import rclpy
from rcl_interfaces.srv import ListParameters
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node

from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal import ros_loader, services
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.client import Client
    from rclpy.service import Service

    from rosbridge_library.internal.type_support import ROSMessage


def populate_random_args(d: object) -> object:
    # Given a dictionary d, replaces primitives with random values
    if isinstance(d, dict):
        for x in d:
            d[x] = populate_random_args(d[x])
        return d
    if isinstance(d, str):
        return str(random.random())
    if isinstance(d, bool):
        return True
    if isinstance(d, int):
        return random.randint(100, 200)
    if isinstance(d, float):
        return 3.5
    return d


class ServiceTester:
    def __init__(self, executor: Executor, name: str, srv_type: str) -> None:
        self.name = name
        self.executor = executor
        self.node = Node("service_tester_" + srv_type.replace("/", "_"))
        self.executor.add_node(self.node)
        self.srvClass = ros_loader.get_service_class(srv_type)
        self.service: Service = self.node.create_service(self.srvClass, name, self.callback)

    def __del__(self) -> None:
        self.executor.remove_node(self.node)

    def start(self) -> None:
        req = self.srvClass.Request()
        gen = populate_random_args(c.extract_values(req))
        assert isinstance(gen, dict)
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

    def callback(self, req: ROSMessage, res: ROSMessage) -> ROSMessage:
        self.req = req
        time.sleep(0.1)
        gen = populate_random_args(c.extract_values(res))
        assert isinstance(gen, dict)
        try:
            res = c.populate_instance(gen, res)
        except:  # Will print() and raise
            print("populating instance")
            print(res)
            print("populating with")
            print(gen)
            raise
        self.output = gen
        return res

    def success(self, rsp: dict[str, Any]) -> None:
        self.rsp = rsp

    def error(self, exc: Exception) -> None:
        self.exc = exc

    def validate(self, equality_function: Callable[[object, object], None]) -> None:
        if hasattr(self, "exc"):
            print(self.exc)
            raise self.exc
        equality_function(self.input, c.extract_values(self.req))
        equality_function(self.output, self.rsp)


class TestServices(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_node")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self) -> None:
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        rclpy.shutdown()

    def msgs_equal(self, msg1: object, msg2: object) -> None:
        if isinstance(msg1, str) and isinstance(msg2, str):
            pass
        else:
            self.assertEqual(type(msg1), type(msg2))
        if type(msg1) in c.list_types:
            assert isinstance(msg1, c.list_types) and isinstance(msg2, c.list_types)
            for x, y in zip(msg1, msg2, strict=False):
                self.msgs_equal(x, y)
        elif (
            type(msg1) in c.primitive_types
            or type(msg1) is str
            or np.issubdtype(type(msg1), np.number)
        ):
            self.assertEqual(msg1, msg2)
        else:
            assert isinstance(msg1, dict) and isinstance(msg2, dict)
            for x in msg1:
                self.assertTrue(x in msg2)
            for x in msg2:
                self.assertTrue(x in msg1)
            for x in msg1:
                self.msgs_equal(msg1[x], msg2[x])

    def test_populate_request_args(self) -> None:
        # Test empty messages
        for srv_type in ["TestEmpty", "TestResponseOnly"]:
            cls = ros_loader.get_service_class("rosbridge_test_msgs/" + srv_type)
            args: Any
            for args in [[], {}, None]:
                # Should throw no exceptions
                services.args_to_service_request_instance(cls.Request(), args)

        # Test msgs with data message
        for srv_type in ["TestRequestOnly", "TestRequestAndResponse"]:
            cls = ros_loader.get_service_class("rosbridge_test_msgs/" + srv_type)
            for args in [[3], {"data": 3}]:
                # Should throw no exceptions
                services.args_to_service_request_instance(cls.Request(), args)
            self.assertRaises(
                FieldTypeMismatchException,
                services.args_to_service_request_instance,
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
            services.args_to_service_request_instance(cls.Request(), args)

    def test_service_call(self) -> None:
        """Test a simple list_parameters service call."""
        # Prepare parameter
        self.node.declare_parameter("test_parameter", 1.0)

        # First, call the service the 'proper' way
        p: Client[ListParameters.Request, ListParameters.Response] = self.node.create_client(
            ListParameters,  # type: ignore[arg-type]
            self.node.get_name() + "/list_parameters",
        )
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

        res = ret.result()
        assert res is not None

        result = res.result

        for x, y in zip(result.names, json_ret["result"]["names"], strict=False):
            self.assertEqual(x, y)

    def test_service_caller(self) -> None:
        """Same as test_service_call but via the thread caller."""
        # Prepare parameter
        self.node.declare_parameter("test_parameter", 1.0)

        # First, call the service the 'proper' way
        p: Client[ListParameters.Request, ListParameters.Response] = self.node.create_client(
            ListParameters,  # type: ignore[arg-type]
            self.node.get_name() + "/list_parameters",
        )
        p.wait_for_service(0.5)
        ret = p.call_async(ListParameters.Request())
        while not ret.done():
            time.sleep(0.1)
        self.node.destroy_client(p)

        rcvd: dict[str, Any] = {"json": None}

        def success(json: dict[str, Any]) -> None:
            rcvd["json"] = json

        def error(exc: Exception) -> NoReturn:
            raise exc

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

        res = ret.result()
        assert res is not None

        result = res.result

        for x, y in zip(result.names, rcvd["json"]["result"]["names"], strict=False):
            self.assertEqual(x, y)

    def test_service_tester(self) -> None:
        t = ServiceTester(
            self.executor, "/test_service_tester", "rosbridge_test_msgs/TestRequestAndResponse"
        )
        t.start()
        time.sleep(0.2)
        t.validate(self.msgs_equal)

    def test_service_tester_alltypes(self) -> None:
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

    def test_random_service_types(self) -> None:
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


if __name__ == "__main__":
    unittest.main()
