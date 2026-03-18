#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from threading import Thread
from typing import TYPE_CHECKING, Any

import numpy as np
import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.internal import actions, message_conversion, ros_loader
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle
    from rclpy.executors import Executor


class ActionTester:
    def __init__(self, executor: Executor) -> None:
        self.executor = executor
        self.node = Node("action_tester")
        self.executor.add_node(self.node)
        self.action_server = ActionServer(
            self.node,
            Fibonacci,
            "get_fibonacci_sequence",
            self.execute_callback,
        )

    def __del__(self) -> None:
        self.executor.remove_node(self.node)

    def execute_callback(self, goal: ServerGoalHandle) -> Fibonacci.Result:
        self.goal = goal
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal.publish_feedback(feedback_msg)  # type: ignore[arg-type] # rclpy type hint is incorrect
            time.sleep(0.1)

        goal.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

    def success(self, rsp: dict[str, Any]) -> None:
        self.rsp = rsp

    def error(self, exc: Exception) -> None:
        self.exc = exc


class TestActions(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        message_conversion.configure()

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
        if type(msg1) in message_conversion.list_types:
            assert isinstance(msg1, message_conversion.list_types) and isinstance(
                msg2, message_conversion.list_types
            )
            for x, y in zip(msg1, msg2, strict=False):
                self.msgs_equal(x, y)
        elif (
            type(msg1) in message_conversion.primitive_types
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

    def test_populate_goal_args(self) -> None:
        # Test empty messages
        for action_type in ["TestEmpty", "TestFeedbackAndResult", "TestResultOnly"]:
            cls = ros_loader.get_action_class("rosbridge_test_msgs/" + action_type)
            args: Any
            for args in [[], {}, None]:
                # Should throw no exceptions
                actions.args_to_action_goal_instance(cls.Goal(), args)

        # Test actions with data message
        for action_type in ["TestGoalOnly", "TestGoalAndResult", "TestGoalFeedbackAndResult"]:
            cls = ros_loader.get_action_class("rosbridge_test_msgs/" + action_type)
            for args in [[3], {"data": 3}]:
                # Should throw no exceptions
                actions.args_to_action_goal_instance(cls.Goal(), args)
            self.assertRaises(
                FieldTypeMismatchException,
                actions.args_to_action_goal_instance,
                cls.Goal(),
                ["hello"],
            )

        # Test actions with multiple fields
        cls = ros_loader.get_action_class("rosbridge_test_msgs/TestMultipleGoalFields")
        for args in [
            [3, 3.5, "hello", False],
            {"int_value": 3, "float_value": 3.5, "string": "hello", "bool_value": False},
        ]:
            # Should throw no exceptions
            actions.args_to_action_goal_instance(cls.Goal(), args)


if __name__ == "__main__":
    unittest.main()
