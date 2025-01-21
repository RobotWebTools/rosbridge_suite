#!/usr/bin/env python
import time
import unittest
from json import dumps, loads
from threading import Thread

import rclpy
from rclpy.node import Node
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.capabilities.service_response import ServiceResponse
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.protocol import Protocol


class TestServiceCapabilities(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("test_service_capabilities")

        self.node.declare_parameter("call_services_in_new_thread", False)
        self.node.declare_parameter("default_call_service_timeout", 5.0)
        self.node.declare_parameter("send_action_goals_in_new_thread", False)

        self.proto = Protocol(self._testMethodName, self.node)
        # change the log function so we can verify errors are logged
        self.proto.log = self.mock_log
        # change the send callback so we can access the rosbridge messages
        # being sent
        self.proto.send = self.local_send_cb
        self.advertise = AdvertiseService(self.proto)
        self.unadvertise = UnadvertiseService(self.proto)
        self.response = ServiceResponse(self.proto)
        self.call_service = CallService(self.proto)
        self.received_message = None
        self.log_entries = []

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def local_send_cb(self, msg):
        self.received_message = msg

    def mock_log(self, loglevel, message, _=None):
        self.log_entries.append((loglevel, message))

    def test_advertise_missing_arguments(self):
        advertise_msg = loads(dumps({"op": "advertise_service"}))
        self.assertRaises(MissingArgumentException, self.advertise.advertise_service, advertise_msg)

    def test_advertise_invalid_arguments(self):
        advertise_msg = loads(dumps({"op": "advertise_service", "type": 42, "service": None}))
        self.assertRaises(InvalidArgumentException, self.advertise.advertise_service, advertise_msg)

    def test_response_missing_arguments(self):
        response_msg = loads(dumps({"op": "service_response"}))
        self.assertRaises(MissingArgumentException, self.response.service_response, response_msg)

        # this message has the optional fields, with correct types, but not the
        # required ones
        response_msg = loads(
            dumps({"op": "service_response", "id": "dummy_service", "values": "none"})
        )
        self.assertRaises(MissingArgumentException, self.response.service_response, response_msg)

    def test_response_invalid_arguments(self):
        response_msg = loads(dumps({"op": "service_response", "service": 5, "result": "error"}))
        self.assertRaises(InvalidArgumentException, self.response.service_response, response_msg)

    def test_advertise_service(self):
        service_path = "/set_bool_1"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.advertise.advertise_service(advertise_msg)

    def test_call_advertised_service(self):
        # Advertise the service
        service_path = "/set_bool_2"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.received_message = None
        self.advertise.advertise_service(advertise_msg)

        # Call the advertised service using rosbridge
        self.received_message = None
        call_msg = loads(
            dumps(
                {
                    "op": "call_service",
                    "id": "foo",
                    "service": service_path,
                    "args": {"data": True},
                }
            )
        )
        Thread(target=self.call_service.call_service, args=(call_msg,)).start()

        start_time = time.monotonic()
        while self.received_message is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start_time > 0.3:
                self.fail("Timed out waiting for service call message.")

        self.assertFalse(self.received_message is None)
        self.assertTrue("op" in self.received_message)
        self.assertEqual(self.received_message["op"], "call_service")
        self.assertTrue("id" in self.received_message)

        # Now send the response
        response_msg = loads(
            dumps(
                {
                    "op": "service_response",
                    "service": service_path,
                    "id": self.received_message["id"],
                    "values": {"success": True, "message": "set bool to true"},
                    "result": True,
                }
            )
        )
        self.received_message = None
        self.response.service_response(response_msg)

        start_time = time.monotonic()
        while self.received_message is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start_time > 0.3:
                self.fail("Timed out waiting for service response message.")

        self.assertFalse(self.received_message is None)
        self.assertEqual(self.received_message["op"], "service_response")
        self.assertTrue(self.received_message["result"])

    @unittest.skip("Gets stuck in rclpy.shutdown on teardown")
    def test_call_advertised_service_with_timeout(self):
        # Advertise the service
        service_path = "/set_bool_3"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.received_message = None
        self.advertise.advertise_service(advertise_msg)

        # Call the advertised service using rosbridge
        self.received_message = None
        call_msg = loads(
            dumps(
                {
                    "op": "call_service",
                    "id": "foo",
                    "service": service_path,
                    "args": {"data": True},
                    "timeout": 0.5,
                }
            )
        )
        Thread(target=self.call_service.call_service, args=(call_msg,)).start()

        start_time = time.monotonic()
        while self.received_message is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start_time > 0.3:
                self.fail("Timed out waiting for service call message.")

        self.assertFalse(self.received_message is None)
        self.assertTrue("op" in self.received_message)
        self.assertEqual(self.received_message["op"], "call_service")
        self.assertTrue("id" in self.received_message)

        self.received_message = None

        start_time = time.monotonic()
        while self.received_message is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start_time > 1.0:
                self.fail("Timed out waiting for service response message.")

        self.assertFalse(self.received_message is None)
        self.assertEqual(self.received_message["op"], "service_response")
        self.assertFalse(self.received_message["result"])
        self.assertEqual(
            self.received_message["values"], "Timeout exceeded while waiting for service response"
        )

    def test_unadvertise_with_live_request(self):
        # Advertise the service
        service_path = "/set_bool_3"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_service",
                    "type": "std_srvs/SetBool",
                    "service": service_path,
                }
            )
        )
        self.received_message = None
        self.advertise.advertise_service(advertise_msg)

        # Now send the response
        call_msg = loads(
            dumps(
                {
                    "op": "call_service",
                    "id": "foo",
                    "service": service_path,
                    "args": {"data": True},
                }
            )
        )
        self.received_message = None
        Thread(target=self.call_service.call_service, args=(call_msg,)).start()

        start_time = time.monotonic()
        while self.received_message is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - start_time > 0.3:
                self.fail("Timed out waiting for service call message.")

        self.assertFalse(self.received_message is None)
        self.assertTrue("op" in self.received_message)
        self.assertEqual(self.received_message["op"], "call_service")
        self.assertTrue("id" in self.received_message)

        # Now unadvertise the service
        # TODO: This raises an exception, likely because of the following rclpy issue:
        # https://github.com/ros2/rclpy/issues/1098
        unadvertise_msg = loads(dumps({"op": "unadvertise_service", "service": service_path}))
        self.received_message = None
        self.unadvertise.unadvertise_service(unadvertise_msg)

        with self.assertRaises(RuntimeError) as context:
            start_time = time.monotonic()
            while self.received_message is None:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if time.monotonic() - start_time > 0.3:
                    self.fail("Timed out waiting for unadvertise service message.")

            self.assertTrue(f"Service {service_path} was unadvertised" in context.exception)


if __name__ == "__main__":
    unittest.main()
