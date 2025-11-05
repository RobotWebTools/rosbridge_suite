#!/usr/bin/env python3
from __future__ import annotations

import time
import unittest
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rosbridge_library.internal.subscription_modifiers import (
    MessageHandler,
    QueueMessageHandler,
    ThrottleMessageHandler,
)


class TestMessageHandlers(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_subscription_modifiers")
        self.executor.add_node(self.node)

    def tearDown(self) -> None:
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    def dummy_cb(self, msg: str) -> None:
        pass

    def test_default_message_handler(self) -> None:
        handler = MessageHandler(None, self.dummy_cb)
        self.help_test_default(handler)

    def test_throttle_message_handler(self) -> None:
        handler: MessageHandler = QueueMessageHandler(MessageHandler(None, self.dummy_cb))
        self.help_test_throttle(handler, 50)

    def test_queue_message_handler_passes_msgs(self) -> None:
        handler: MessageHandler = QueueMessageHandler(MessageHandler(None, self.dummy_cb))
        self.help_test_queue(handler, 1000)
        handler.finish()

    def test_queue_message_handler_stops(self) -> None:
        received: dict[str, Any] = {"msgs": []}

        def cb(msg: str) -> None:
            received["msgs"].append(msg)

        handler: QueueMessageHandler = QueueMessageHandler(MessageHandler(None, cb))

        self.assertTrue(handler.is_alive())

        handler.finish()

        self.assertFalse(handler.is_alive())

    def test_queue_message_handler_queue(self) -> None:
        received: dict[str, Any] = {"msgs": []}

        def cb(msg: str) -> None:
            received["msgs"].append(msg)

        msgs = range(1000)

        handler: MessageHandler = MessageHandler(None, cb)

        handler = handler.set_throttle_rate(10000)
        handler = handler.set_queue_length(10)
        self.assertIsInstance(handler, QueueMessageHandler)

        # 'hello' is handled immediately
        handler.handle_message("hello")
        time.sleep(0.02)
        # queue is now empty, but throttling is in effect
        # no messages will be handled in the next 10 seconds

        # these will fill up the queue, with newer values displacing old ones
        # nothing gets sent because the throttle rate
        for x in msgs:
            handler.handle_message(x)

        handler = handler.set_throttle_rate(0)

        time.sleep(0.1)

        try:
            self.assertEqual(["hello", *list(range(990, 1000))], received["msgs"])
        finally:
            handler.finish()

    def test_queue_message_handler_dropping(self) -> None:
        received: dict[str, Any] = {"msgs": []}

        def cb(msg: int) -> None:
            received["msgs"].append(msg)
            time.sleep(1)

        queue_length = 5
        msgs = range(queue_length * 5)

        handler = MessageHandler(None, cb)

        handler = handler.set_queue_length(queue_length)
        self.assertIsInstance(handler, QueueMessageHandler)

        # send all messages at once.
        # only the first and the last queue_length should get through,
        # because the callbacks are blocked.
        for x in msgs:
            handler.handle_message(x)
            # yield the thread so the first callback can append,
            # otherwise the first handled value is non-deterministic.
            time.sleep(0.01)

        # wait long enough for all the callbacks, and then some.
        time.sleep(queue_length + 3)

        try:
            self.assertEqual([msgs[0], *list(msgs[-queue_length:])], received["msgs"])
        except:  # Will finish and raise
            handler.finish()
            raise

        handler.finish()

    def test_queue_message_handler_rate(self) -> None:
        handler = MessageHandler(None, self.dummy_cb)
        self.help_test_queue_rate(handler, 50, 10)
        handler.finish()

    # Helper methods for each of the three Handler types, plus one for Queue+Rate.
    # Used in standalone testing as well as the test_transition_functionality test
    def help_test_default(self, handler: MessageHandler) -> MessageHandler:
        handler = handler.set_queue_length(0)
        handler = handler.set_throttle_rate(0)
        self.assertIsInstance(handler, MessageHandler)

        msg = "test_default_message_handler"
        received: dict[str, Any] = {"msg": None}

        def cb_str(msg: str) -> None:
            received["msg"] = msg

        handler.publish = cb_str

        self.assertTrue(handler.time_remaining() == 0)
        t1 = time.monotonic()
        handler.handle_message(msg)
        t2 = time.monotonic()

        self.assertEqual(received["msg"], msg)
        self.assertLessEqual(t1, handler.last_publish)
        self.assertLessEqual(handler.last_publish, t2)
        self.assertEqual(handler.time_remaining(), 0)

        received = {"msgs": []}

        def cb_int(msg: int) -> None:
            received["msgs"].append(msg)

        handler.publish = cb_int
        xs = list(range(10000))
        for x in xs:
            handler.handle_message(x)

        self.assertEqual(received["msgs"], xs)
        return handler

    def help_test_throttle(self, handler: MessageHandler, throttle_rate: float) -> MessageHandler:
        handler = handler.set_queue_length(0)
        handler = handler.set_throttle_rate(throttle_rate)
        self.assertIsInstance(handler, ThrottleMessageHandler)

        msg = "test_throttle_message_handler"

        # First, try with a single message
        received: dict[str, Any] = {"msg": None}

        def cb_str(msg: str) -> None:
            received["msg"] = msg

        handler.publish = cb_str

        # ensure the handler doesn't swallow this message
        time.sleep(2.0 * handler.throttle_rate)
        handler.handle_message(msg)
        self.assertEqual(received["msg"], msg)

        # sleep to make sure the handler sends right away for the second part
        time.sleep(2.0 * handler.throttle_rate)

        received = {"msgs": []}

        def cb_int(msg: int) -> None:
            received["msgs"].append(msg)

        handler.publish = cb_int
        x = 0
        time_padding = handler.throttle_rate / 4.0
        for i in range(1, 10):
            # We guarantee that in the while loop below only the first message is handled
            # All subsequent messages (within throttling window - time_padding ) are dropped
            # Time padding is a test-only hack around race condition when time.time() - fin is within
            # the throttling window, but handler.handle_message(x) gets a later timestamp that is outside.
            time.sleep(2.0 * time_padding)
            fin = time.time() + throttle_rate / 1000.0 - time_padding
            while time.time() < fin:
                handler.handle_message(x)
                x = x + 1
            self.assertEqual(len(received["msgs"]), i)
        return handler

    def help_test_queue(self, handler: MessageHandler, queue_length: int) -> MessageHandler:
        handler = handler.set_queue_length(queue_length)
        self.assertIsInstance(handler, QueueMessageHandler)

        received: dict[str, Any] = {"msgs": []}

        def cb(msg: int) -> None:
            received["msgs"].append(msg)

        handler.publish = cb

        msgs = list(range(queue_length))
        for x in msgs:
            handler.handle_message(x)

        time.sleep(0.1)

        self.assertEqual(msgs, received["msgs"])
        return handler

    def help_test_queue_rate(
        self, handler: MessageHandler, throttle_rate: float, queue_length: int
    ) -> MessageHandler:
        handler = handler.set_throttle_rate(throttle_rate)
        handler = handler.set_queue_length(queue_length)
        self.assertIsInstance(handler, QueueMessageHandler)

        received: dict[str, Any] = {"msg": None}

        def cb(msg: int) -> None:
            received["msg"] = msg

        handler.publish = cb

        throttle_rate_sec = throttle_rate / 1000.0

        # ensure previous tests' last sent time is long enough ago
        time.sleep(throttle_rate_sec)
        for x in range(queue_length):
            handler.handle_message(x)

        time.sleep(throttle_rate_sec / 2.0)

        try:
            for x in range(10):
                self.assertEqual(x, received["msg"])
                time.sleep(throttle_rate_sec)
        except:  # Will finish and raise
            handler.finish()
            raise

        return handler

    # Test that each transition works and is stable
    def test_transitions(self) -> None:
        # MessageHandler.transition is stable
        handler = MessageHandler(None, self.dummy_cb)
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)

        # Going from MessageHandler to ThrottleMessageHandler...
        handler = MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_throttle_rate(100)
        self.assertIsInstance(next_handler, ThrottleMessageHandler)
        handler = next_handler
        # Testing transition returns another ThrottleMessageHandler
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)
        # And finally going back to MessageHandler
        next_handler = handler.set_throttle_rate(0)
        self.assertIsInstance(next_handler, MessageHandler)

        # Same for QueueMessageHandler
        handler = MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_queue_length(100)
        self.assertIsInstance(next_handler, QueueMessageHandler)
        handler = next_handler
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)
        next_handler = handler.set_queue_length(0)
        self.assertIsInstance(next_handler, MessageHandler)

        # Checking a QueueMessageHandler with rate limit can be generated both ways
        handler = MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_queue_length(100).set_throttle_rate(100)
        self.assertIsInstance(next_handler, QueueMessageHandler)
        next_handler.finish()
        next_handler = handler.set_throttle_rate(100).set_queue_length(100)
        self.assertIsInstance(next_handler, QueueMessageHandler)
        next_handler.finish()
        handler = next_handler
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)
        # Check both steps on the way back to plain MessageHandler
        next_handler = handler.set_throttle_rate(0)
        self.assertIsInstance(next_handler, QueueMessageHandler)
        next_handler = handler.set_queue_length(0)
        self.assertIsInstance(next_handler, MessageHandler)

    def test_transition_functionality(self) -> None:
        # Test individually
        handler: MessageHandler = MessageHandler(None, None)
        handler = self.help_test_queue(handler, 10)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler.finish()

        # Test combinations
        handler = MessageHandler(None, None)
        handler = self.help_test_queue(handler, 10)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_default(handler)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_queue(handler, 10)
        handler = self.help_test_default(handler)
        handler = self.help_test_throttle(handler, 50)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler = self.help_test_default(handler)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_default(handler)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler = self.help_test_queue(handler, 10)
        handler = self.help_test_throttle(handler, 50)
        handler.finish()

        # Test duplicates
        handler = MessageHandler(None, None)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler = self.help_test_queue_rate(handler, 100, 10)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_throttle(handler, 100)
        handler.finish()

        handler = MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler = self.help_test_default(handler)
        handler.finish()


if __name__ == "__main__":
    unittest.main()
