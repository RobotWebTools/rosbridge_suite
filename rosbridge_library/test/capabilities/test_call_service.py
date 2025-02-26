#!/usr/bin/env python
import time
import unittest
from json import dumps, loads
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.protocol import Protocol
from std_srvs.srv import SetBool, Trigger


class TestCallService(unittest.TestCase):
    def trigger_cb(self, request, response):
        """Helper callback function for a test service with no arguments."""
        response.success = True
        response.message = "called trigger service successfully"
        return response

    def trigger_long_cb(self, request, response):
        """Helper callback function for a long running test service with no arguments."""
        time.sleep(0.5)
        response.success = True
        response.message = "called trigger service successfully"
        return response

    def set_bool_cb(self, request, response):
        """Helper callback function for a test service with arguments."""
        response.success = request.data
        if request.data:
            response.message = "set bool to true"
        else:
            response.message = "set bool to false"
        return response

    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_call_service")
        self.executor.add_node(self.node)

        self.node.declare_parameter("call_services_in_new_thread", False)
        self.node.declare_parameter("default_call_service_timeout", 5.0)
        self.node.declare_parameter("send_action_goals_in_new_thread", False)

        # Create service servers with a separate callback group
        self.cb_group = ReentrantCallbackGroup()
        self.trigger_srv = self.node.create_service(
            Trigger,
            self.node.get_name() + "/trigger",
            self.trigger_cb,
            callback_group=self.cb_group,
        )
        self.trigger_long_srv = self.node.create_service(
            Trigger,
            self.node.get_name() + "/trigger_long",
            self.trigger_long_cb,
            callback_group=self.cb_group,
        )
        self.set_bool_srv = self.node.create_service(
            SetBool,
            self.node.get_name() + "/set_bool",
            self.set_bool_cb,
            callback_group=self.cb_group,
        )

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.exec_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_missing_arguments(self):
        proto = Protocol("test_missing_arguments", self.node)
        s = CallService(proto)
        msg = loads(dumps({"op": "call_service"}))
        self.assertRaises(MissingArgumentException, s.call_service, msg)

    def test_invalid_arguments(self):
        proto = Protocol("test_invalid_arguments", self.node)
        s = CallService(proto)

        msg = loads(dumps({"op": "call_service", "service": 3}))
        self.assertRaises(InvalidArgumentException, s.call_service, msg)

    def test_call_service_works(self):
        client = self.node.create_client(Trigger, self.trigger_srv.srv_name)
        assert client.wait_for_service(1.0)

        proto = Protocol("test_call_service_works", self.node)
        s = CallService(proto)
        send_msg = loads(dumps({"op": "call_service", "service": self.trigger_srv.srv_name}))

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None, compression="none"):
            print(msg)
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(send_msg)

        self.assertTrue(received["arrived"])
        values = received["msg"]["values"]
        self.assertEqual(values["success"], True)
        self.assertEqual(values["message"], "called trigger service successfully")

    def test_call_service_args(self):
        client = self.node.create_client(SetBool, self.set_bool_srv.srv_name)
        assert client.wait_for_service(1.0)

        proto = Protocol("test_call_service_args", self.node)
        s = CallService(proto)
        send_msg = loads(
            dumps(
                {
                    "op": "call_service",
                    "service": self.set_bool_srv.srv_name,
                    "args": {"data": True},
                }
            )
        )

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None, compression="none"):
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(send_msg)

        self.assertTrue(received["arrived"])
        values = received["msg"]["values"]
        self.assertEqual(values["success"], True)
        self.assertEqual(values["message"], "set bool to true")

    def test_call_service_fails(self):

        client = self.node.create_client(Trigger, self.trigger_srv.srv_name)
        assert client.wait_for_service(1.0)

        proto = Protocol("test_call_service_works", self.node)
        s = CallService(proto)
        send_msg = loads(
            dumps(
                {
                    "op": "call_service",
                    "service": self.set_bool_srv.srv_name,
                    "args": {"data": 42.0},  # This data type is wrong so it will fail
                }
            )
        )

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None, compression="none"):
            print(msg)
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(send_msg)

        self.assertTrue(received["arrived"])
        self.assertFalse(received["msg"]["result"])

    def test_call_service_timeout(self):

        client = self.node.create_client(Trigger, self.trigger_long_srv.srv_name)
        assert client.wait_for_service(1.0)

        proto = Protocol("test_call_service_timeout", self.node)
        s = CallService(proto)
        send_msg = loads(
            dumps({"op": "call_service", "service": self.trigger_long_srv.srv_name, "timeout": 2.0})
        )

        received = {"msg": None, "arrived": False}

        def cb(msg, cid=None, compression="none"):
            print("Received message")
            received["msg"] = msg
            received["arrived"] = True

        proto.send = cb

        s.call_service(send_msg)

        self.assertTrue(received["arrived"])
        self.assertTrue(received["msg"]["result"])
        values = received["msg"]["values"]
        self.assertEqual(values["success"], True)
        self.assertEqual(values["message"], "called trigger service successfully")

        send_msg = loads(
            dumps({"op": "call_service", "service": self.trigger_long_srv.srv_name, "timeout": 0.1})
        )
        received = {"msg": None, "arrived": False}

        s.call_service(send_msg)

        self.assertTrue(received["arrived"])
        self.assertFalse(received["msg"]["result"])
        values = received["msg"]["values"]
        self.assertEqual(values, "Timeout exceeded while waiting for service response")
