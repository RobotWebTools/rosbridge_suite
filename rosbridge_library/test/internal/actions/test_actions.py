#!/usr/bin/env python3
import time
import unittest
from threading import Thread

import numpy as np
import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.internal import actions
from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import FieldTypeMismatchException


class ActionTester:
    def __init__(self, executor):
        self.executor = executor
        self.node = Node("action_tester")
        self.executor.add_node(self.node)
        self.action_server = ActionServer(
            self.node,
            Fibonacci,
            "get_fibonacci_sequence",
            self.execute_callback,
        )

    def __del__(self):
        self.executor.remove_node(self.node)

    def start(self):
        req = self.action_class.Goal()
        gen = c.extract_values(req)
        thread = actions.ActionClientHandler(
            self.name,
            self.name,
            gen,
            self.success,
            self.error,
            self.node,
        )
        thread.start()
        thread.join()

    def execute_callback(self, goal):
        self.goal = goal
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal.request.order):
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

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


class TestActions(unittest.TestCase):
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

    def test_populate_goal_args(self):
        # Test empty messages
        for action_type in ["TestEmpty", "TestFeedbackAndResult", "TestResultOnly"]:
            cls = ros_loader.get_action_class("rosbridge_test_msgs/" + action_type)
            for args in [[], {}, None]:
                # Should throw no exceptions
                actions.args_to_action_goal_instance("", cls.Goal(), args)

        # Test actions with data message
        for action_type in ["TestGoalOnly", "TestGoalAndResult", "TestGoalFeedbackAndResult"]:
            cls = ros_loader.get_action_class("rosbridge_test_msgs/" + action_type)
            for args in [[3], {"data": 3}]:
                # Should throw no exceptions
                actions.args_to_action_goal_instance("", cls.Goal(), args)
            self.assertRaises(
                FieldTypeMismatchException,
                actions.args_to_action_goal_instance,
                "",
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
            actions.args_to_action_goal_instance("", cls.Goal(), args)

    def test_send_action_goal(self):
        """Test a simple action call"""
        ActionTester(self.executor)
        self.result = None

        def get_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                return
            result_future = future.result().get_result_async()
            result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            self.result = future.result().result

        # First, call the action the 'proper' way
        client = ActionClient(self.node, Fibonacci, "get_fibonacci_sequence")
        client.wait_for_server()
        goal = Fibonacci.Goal()
        goal.order = 5
        future = client.send_goal_async(goal)
        future.add_done_callback(get_response_callback)
        while not future.done():
            time.sleep(0.1)
        client.destroy()

        self.assertIsNotNone(self.result)
        self.assertEqual(list(self.result.sequence), [0, 1, 1, 2, 3, 5])

        # Now, call using the services
        json_ret = actions.SendGoal().send_goal(
            self.node,
            "get_fibonacci_sequence",
            "example_interfaces/Fibonacci",
            {"order": 5},
        )
        self.assertEqual(list(json_ret["result"]["sequence"]), [0, 1, 1, 2, 3, 5])

    def test_action_client_handler(self):
        """Same as test_service_call but via the thread caller"""
        ActionTester(self.executor)

        received = {"json": None}

        def success(json):
            received["json"] = json

        def error():
            raise Exception()

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
