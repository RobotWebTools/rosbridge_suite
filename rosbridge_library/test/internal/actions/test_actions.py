#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from threading import Thread
from typing import TYPE_CHECKING, Any, NoReturn

import numpy as np
import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rosbridge_library.internal import actions, ros_loader
from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException

if TYPE_CHECKING:
    from rclpy.action.client import ClientGoalHandle
    from rclpy.action.server import ServerGoalHandle
    from rclpy.executors import Executor
    from rclpy.task import Future
    from rclpy.type_support import GetResultServiceResponse


class ActionTester:
    def __init__(self, executor: Executor) -> None:
        self.executor = executor
        self.node = Node("action_tester")
        self.executor.add_node(self.node)
        self.action_server: ActionServer[Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback] = (
            ActionServer(
                self.node,
                Fibonacci,  # type: ignore[arg-type]
                "get_fibonacci_sequence",
                self.execute_callback,
            )
        )

    def __del__(self) -> None:
        self.executor.remove_node(self.node)

    def execute_callback(
        self, goal: ServerGoalHandle[Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback]
    ) -> Fibonacci.Result:
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

    def test_send_action_goal(self) -> None:
        """Test a simple action call."""
        ActionTester(self.executor)
        received: dict[str, Any] = {"msg": None}

        def get_response_callback(future: Future[ClientGoalHandle]) -> None:
            goal_handle = future.result()
            assert goal_handle is not None
            if not goal_handle.accepted:
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(get_result_callback)

        def get_result_callback(future: Future[GetResultServiceResponse[Fibonacci.Result]]) -> None:
            response = future.result()
            assert response is not None
            received["msg"] = response.result

        # First, call the action the 'proper' way
        client: ActionClient[Fibonacci.Goal, Fibonacci.Result, Fibonacci.Feedback] = ActionClient(
            self.node,
            Fibonacci,  # type: ignore[arg-type]
            "get_fibonacci_sequence",
        )
        client.wait_for_server()
        goal = Fibonacci.Goal()
        goal.order = 5
        future = client.send_goal_async(goal)
        future.add_done_callback(get_response_callback)
        while not future.done():
            time.sleep(0.1)
        client.destroy()

        self.assertIsNotNone(received["msg"])
        self.assertEqual(list(received["msg"].sequence), [0, 1, 1, 2, 3, 5])

        # Now, call using the services
        json_ret = actions.SendGoal().send_goal(
            self.node,
            "get_fibonacci_sequence",
            "example_interfaces/Fibonacci",
            {"order": 5},
        )
        self.assertEqual(list(json_ret["result"]["sequence"]), [0, 1, 1, 2, 3, 5])

    def test_action_client_handler(self) -> None:
        """Test service_call via the thread caller."""
        ActionTester(self.executor)

        received: dict[str, Any] = {"json": None}

        def success(json: dict[str, Any]) -> None:
            received["json"] = json

        def error(exc: Exception) -> NoReturn:
            raise exc

        # Now, call using the services
        order = 5
        actions.ActionClientHandler(
            "get_fibonacci_sequence",
            "example_interfaces/Fibonacci",
            {"order": order},
            success,
            error,
            None,  # No feedback
            self.node,
        ).start()

        time.sleep(1.0)

        self.assertIsNotNone(received["json"])
        self.assertEqual(list(received["json"]["result"]["sequence"]), [0, 1, 1, 2, 3, 5])


if __name__ == "__main__":
    unittest.main()
