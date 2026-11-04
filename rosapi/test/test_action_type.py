from __future__ import annotations

import time
import unittest
from threading import Thread
from typing import TYPE_CHECKING

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rosapi import proxy

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle

# Actions are discovered asynchronously, so the happy-path lookup is retried.
DISCOVERY_TIMEOUT_SEC = 10.0
DISCOVERY_POLL_SEC = 0.1


class TestActionType(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.node = Node("test_action_type")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        proxy.init(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self) -> None:
        self.executor.remove_node(self.node)
        self.executor.shutdown()
        self.exec_thread.join()
        self.node.destroy_node()
        rclpy.shutdown()

    def execute_callback(self, goal_handle: ServerGoalHandle) -> Fibonacci.Result:
        goal_handle.succeed()
        return Fibonacci.Result()

    def test_get_action_type_of_missing_action(self) -> None:
        # rosapi passes a plain rclpy node, so a lookup going through
        # ros2action.api used to raise AttributeError here instead of
        # reporting the action as missing.
        self.assertEqual(proxy.get_action_type("/no_such_action"), "")

    def test_get_action_type(self) -> None:
        action_server = ActionServer(self.node, Fibonacci, "/fibonacci", self.execute_callback)
        try:
            action_type = ""
            deadline = time.monotonic() + DISCOVERY_TIMEOUT_SEC
            while not action_type and time.monotonic() < deadline:
                action_type = proxy.get_action_type("/fibonacci")
                if not action_type:
                    time.sleep(DISCOVERY_POLL_SEC)

            self.assertEqual(action_type, "example_interfaces/action/Fibonacci")
        finally:
            action_server.destroy()
