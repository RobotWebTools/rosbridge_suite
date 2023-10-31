#!/usr/bin/env python
import unittest
from json import dumps, loads

import rclpy
from rclpy.node import Node
from rosbridge_library.capabilities.action_result import ActionResult
from rosbridge_library.capabilities.advertise_action import AdvertiseAction
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.protocol import Protocol


class TestActionCapabilities(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = Node("test_action_capabilities")

        self.node.declare_parameter("call_services_in_new_thread", False)

        self.proto = Protocol(self._testMethodName, self.node)
        # change the log function so we can verify errors are logged
        self.proto.log = self.mock_log
        # change the send callback so we can access the rosbridge messages
        # being sent
        self.proto.send = self.local_send_cb
        self.advertise = AdvertiseAction(self.proto)
        # self.unadvertise = UnadvertiseService(self.proto)
        self.result = ActionResult(self.proto)
        # self.call_service = CallService(self.proto)
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
        advertise_msg = loads(dumps({"op": "advertise_action"}))
        self.assertRaises(MissingArgumentException, self.advertise.advertise_action, advertise_msg)

    def test_advertise_invalid_arguments(self):
        advertise_msg = loads(dumps({"op": "advertise_action", "type": 42, "action": None}))
        self.assertRaises(InvalidArgumentException, self.advertise.advertise_action, advertise_msg)

    def test_result_missing_arguments(self):
        result_msg = loads(dumps({"op": "action_result"}))
        self.assertRaises(MissingArgumentException, self.result.action_result, result_msg)

        # this message has the optional fields, with correct types, but not the
        # required ones
        result_msg = loads(dumps({"op": "action_result", "id": "dummy_action", "values": "none"}))
        self.assertRaises(MissingArgumentException, self.result.action_result, result_msg)

    def test_result_invalid_arguments(self):
        result_msg = loads(dumps({"op": "action_result", "action": 5, "result": "error"}))
        self.assertRaises(InvalidArgumentException, self.result.action_result, result_msg)

    def test_advertise_action(self):
        action_path = "/fibonacci_1"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_action",
                    "type": "example_interfaces/Fibonacci",
                    "action": action_path,
                }
            )
        )
        self.advertise.advertise_action(advertise_msg)

    def test_execute_advertised_action(self):
        # Advertise the action
        action_path = "/fibonacci_2"
        advertise_msg = loads(
            dumps(
                {
                    "op": "advertise_action",
                    "type": "example_interfaces/Fibonacci",
                    "action": action_path,
                }
            )
        )
        self.received_message = None
        self.advertise.advertise_action(advertise_msg)

        # TODO: Fill out the rest


if __name__ == "__main__":
    unittest.main()
