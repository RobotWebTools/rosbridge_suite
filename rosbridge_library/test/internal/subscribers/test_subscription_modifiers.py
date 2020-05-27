#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest
import time

from rosbridge_library.internal import subscription_modifiers as subscribe


class TestMessageHandlers(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_message_handlers")

    def dummy_cb(self, msg):
        pass

    def test_default_message_handler(self):
        handler = subscribe.MessageHandler(None, self.dummy_cb)
        self.help_test_default(handler)

    def test_throttle_message_handler(self):
        handler = subscribe.ThrottleMessageHandler(subscribe.MessageHandler(None, self.dummy_cb))
        self.help_test_throttle(handler, 50)

    def test_queue_message_handler_passes_msgs(self):
        handler = subscribe.QueueMessageHandler(subscribe.MessageHandler(None, self.dummy_cb))
        self.help_test_queue(handler, 1000)
        handler.finish()

    def test_queue_message_handler_stops(self):
        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)

        handler = subscribe.QueueMessageHandler(subscribe.MessageHandler(None, cb))

        self.assertTrue(handler.is_alive())

        handler.finish()

        self.assertFalse(handler.is_alive())

    def test_queue_message_handler_queue(self):
        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)

        msgs = range(1000)

        handler = subscribe.MessageHandler(None, cb)

        handler = handler.set_throttle_rate(10000)
        handler = handler.set_queue_length(10)
        self.assertIsInstance(handler, subscribe.QueueMessageHandler)

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
            self.assertEqual(["hello"] + list(range(990, 1000)), received["msgs"])
        except:
            handler.finish()
            raise

        handler.finish()

    def test_queue_message_handler_dropping(self):
        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)
            time.sleep(1)

        queue_length = 5
        msgs = list(range(queue_length * 5))

        handler = subscribe.MessageHandler(None, cb)

        handler = handler.set_queue_length(queue_length)
        self.assertIsInstance(handler, subscribe.QueueMessageHandler)

        # yield the thread to let QueueMessageHandler reach wait().
        time.sleep(0.001)

        # send all messages at once.
        # only the first and the last queue_length should get through,
        # because the callbacks are blocked.
        for x in msgs:
            handler.handle_message(x)
            # yield the thread so the first callback can append,
            # otherwise the first handled value is non-deterministic.
            time.sleep(0.001)

        # wait long enough for all the callbacks, and then some.
        time.sleep(queue_length + 3)

        try:
            self.assertEqual([msgs[0]] + msgs[-queue_length:], received["msgs"])
        except:
            handler.finish()
            raise

        handler.finish()

    def test_queue_message_handler_rate(self):
        handler = subscribe.MessageHandler(None, self.dummy_cb)
        self.help_test_queue_rate(handler, 50, 10)
        handler.finish()

    # Helper methods for each of the three Handler types, plus one for Queue+Rate.
    # Used in standalone testing as well as the test_transition_functionality test
    def help_test_default(self, handler):
        handler = handler.set_queue_length(0)
        handler = handler.set_throttle_rate(0)
        self.assertIsInstance(handler, subscribe.MessageHandler)

        msg = "test_default_message_handler"
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg
        handler.publish = cb

        self.assertTrue(handler.time_remaining() == 0)
        t1 = time.time()
        handler.handle_message(msg)
        t2 = time.time()

        self.assertEqual(received["msg"], msg)
        self.assertLessEqual(t1, handler.last_publish)
        self.assertLessEqual(handler.last_publish, t2)
        self.assertEqual(handler.time_remaining(), 0)

        received = {"msgs": []}
        def cb(msg):
            received["msgs"].append(msg)
        handler.publish = cb
        xs = list(range(10000))
        for x in xs:
            handler.handle_message(x)

        self.assertEqual(received["msgs"], xs)
        return handler

    def help_test_throttle(self, handler, throttle_rate):
        handler = handler.set_queue_length(0)
        handler = handler.set_throttle_rate(throttle_rate)
        self.assertIsInstance(handler, subscribe.ThrottleMessageHandler)

        msg = "test_throttle_message_handler"

        # First, try with a single message
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        handler.publish = cb

        # ensure the handler doesn't swallow this message
        time.sleep(2.0 * handler.throttle_rate)
        handler.handle_message(msg)
        self.assertEqual(received["msg"], msg)

        # sleep to make sure the handler sends right away for the second part
        time.sleep(2.0 * handler.throttle_rate)

        received = {"msgs": []}
        def cb(msg):
            received["msgs"].append(msg)

        handler.publish = cb
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

    def help_test_queue(self, handler, queue_length):
        handler = handler.set_queue_length(queue_length)
        self.assertIsInstance(handler, subscribe.QueueMessageHandler)

        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)

        handler.publish = cb

        msgs = list(range(queue_length))
        for x in msgs:
            handler.handle_message(x)

        time.sleep(0.1)

        self.assertEqual(msgs, received["msgs"])
        return handler

    def help_test_queue_rate(self, handler, throttle_rate, queue_length):
        handler = handler.set_throttle_rate(throttle_rate)
        handler = handler.set_queue_length(queue_length)
        self.assertIsInstance(handler, subscribe.QueueMessageHandler)

        received = {"msg": None}

        def cb(msg):
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
        except:
            handler.finish()
            raise

        return handler

# Test that each transition works and is stable
    def test_transitions(self):
        # MessageHandler.transition is stable
        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)

        # Going from MessageHandler to ThrottleMessageHandler...
        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_throttle_rate(100)
        self.assertIsInstance(next_handler, subscribe.ThrottleMessageHandler)
        handler = next_handler
        # Testing transition returns another ThrottleMessageHandler
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)
        # And finally going back to MessageHandler
        next_handler = handler.set_throttle_rate(0)
        self.assertIsInstance(next_handler, subscribe.MessageHandler)

        # Same for QueueMessageHandler
        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_queue_length(100)
        self.assertIsInstance(next_handler, subscribe.QueueMessageHandler)
        handler = next_handler
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)
        next_handler = handler.set_queue_length(0)
        self.assertIsInstance(next_handler, subscribe.MessageHandler)

        # Checking a QueueMessageHandler with rate limit can be generated both ways
        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_queue_length(100).set_throttle_rate(100)
        self.assertIsInstance(next_handler, subscribe.QueueMessageHandler)
        next_handler.finish()
        next_handler = handler.set_throttle_rate(100).set_queue_length(100)
        self.assertIsInstance(next_handler, subscribe.QueueMessageHandler)
        next_handler.finish()
        handler = next_handler
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)
        # Check both steps on the way back to plain MessageHandler
        next_handler = handler.set_throttle_rate(0)
        self.assertIsInstance(next_handler, subscribe.QueueMessageHandler)
        next_handler = handler.set_queue_length(0)
        self.assertIsInstance(next_handler, subscribe.MessageHandler)

    def test_transition_functionality(self):
        # Test individually
        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_queue(handler, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler.finish()

        # Test combinations
        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_queue(handler, 10)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_default(handler)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_queue(handler, 10)
        handler = self.help_test_default(handler)
        handler = self.help_test_throttle(handler, 50)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler = self.help_test_default(handler)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_default(handler)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler = self.help_test_queue(handler, 10)
        handler = self.help_test_throttle(handler, 50)
        handler.finish()

        # Test duplicates
        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_queue_rate(handler, 50, 10)
        handler = self.help_test_queue_rate(handler, 100, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_throttle(handler, 50)
        handler = self.help_test_throttle(handler, 100)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = self.help_test_default(handler)
        handler = self.help_test_default(handler)
        handler.finish()


#        handler = self.help_test_throttle(handler, 50)
#        handler = self.help_test_default(handler)
#        handler = self.help_test_throttle(handler, 50)
#        handler = self.help_test_default(handler)
#        handler = self.help_test_throttle(handler, 50)


PKG = 'rosbridge_library'
NAME = 'test_message_handlers'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMessageHandlers)
