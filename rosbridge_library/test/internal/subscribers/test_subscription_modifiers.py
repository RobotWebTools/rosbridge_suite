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
        msg = "test_default_message_handler"
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        handler = subscribe.MessageHandler(None, cb)
        self.assertTrue(handler.time_remaining() == 0)
        t1 = time.time()
        handler.handle_message(msg)
        t2 = time.time()

        self.assertEqual(received["msg"], msg)
        self.assertLessEqual(t1, handler.last_publish)
        self.assertLessEqual(handler.last_publish, t2)
        self.assertEqual(handler.time_remaining(), 0)

        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.transition()
        self.assertEqual(handler, next_handler)

        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_throttle_rate(100)
        self.assertIsInstance(next_handler, subscribe.ThrottleMessageHandler)

        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_queue_length(100)
        self.assertIsInstance(next_handler, subscribe.MessageHandler)

        handler = subscribe.MessageHandler(None, self.dummy_cb)
        next_handler = handler.set_queue_length(100).set_throttle_rate(100)
        self.assertIsInstance(next_handler, subscribe.QueueMessageHandler)
        next_handler.finish()

        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)

        handler.publish = cb
        vals = range(10000, 20000)
        for x in vals:
            handler.handle_message(x)

        self.assertEqual(vals, received["msgs"])

    def test_throttle_message_handler(self):
        msg = "test_throttle_message_handler"

        # First, try with a single message
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        handler = subscribe.MessageHandler(None, cb)
        handler = handler.set_throttle_rate(10)
        self.assertIsInstance(handler, subscribe.ThrottleMessageHandler)
        handler.handle_message(msg)
        self.assertEqual(received["msg"], msg)

        # Now, saturate with messages, see how many we get
        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)
        handler.publish = cb

        numsent = 0
        last = -1
        for i in range(1, 10):
            start = time.time()
            while (time.time() - start < handler.throttle_rate):
                handler.handle_message(numsent)
                numsent = numsent + 1

            self.assertGreater(numsent, i)
            self.assertEqual(len(received["msgs"]), i)
            self.assertGreater(received["msgs"][-1], last)
            last = numsent

    def test_queue_message_handler_passes_msgs(self):
        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg)

        handler = subscribe.QueueMessageHandler(subscribe.MessageHandler(None, cb))
        handler.queue_length = 1000

        msgs = range(1000)
        for x in msgs:
            handler.handle_message(x)

        time.sleep(0.1)
        handler.finish()

        self.assertEqual(msgs, received["msgs"])

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
        handler.handle_message("hello")
        handler = handler.set_throttle_rate(10000)
        handler = handler.set_queue_length(10)

        for x in msgs:
            handler.handle_message(x)

        handler.set_throttle_rate(0)

        time.sleep(0.1)

        try:
            self.assertEqual(["hello"] + range(990, 1000), received["msgs"])
        except:
            handler.finish()
            raise

        handler.finish()

    def test_queue_message_handler_rate(self):
        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        handler = subscribe.MessageHandler(None, cb)
        handler = handler.set_throttle_rate(50)
        handler = handler.set_queue_length(10)

        for x in range(10):
            handler.handle_message(x)

        time.sleep(0.025)

        try:
            for x in range(10):
                self.assertEqual(x, received["msg"])
                time.sleep(0.05)
        except:
            handler.finish()
            raise

        handler.finish()

    def test_transitions(self):
        def test_default(handler):
            handler = handler.set_queue_length(0)
            handler = handler.set_throttle_rate(0)
            received = {"msgs": []}
            def cb(msg):
                received["msgs"].append(msg)
            handler.publish = cb
            xs = range(10000)
            for x in xs:
                handler.handle_message(x)

            self.assertEqual(received["msgs"], xs)
            return handler

        def test_throttle(handler, throttle_rate):
            received = {"msgs": []}
            def cb(msg):
                received["msgs"].append(msg)
            handler = handler.set_queue_length(0)
            handler = handler.set_throttle_rate(throttle_rate)
            handler.publish = cb
            x = 0
            time_padding = 0.01
            for i in range(1, 10):
                # We guarantee that in the while loop below only the first message is handled
                # All subsequent messages (within throttling window - time_padding ) are dropped
                # Time padding is a test-only hack around race condition when time.time() - fin is within
                # the throttling window, but handler.handle_message(x) gets a later timestamp that is outside.
                time.sleep(2*time_padding)
                fin = time.time() + throttle_rate / 1000.0 - time_padding
                while time.time() < fin:
                    handler.handle_message(x)
                    x = x + 1
                self.assertEqual(len(received["msgs"]), i)
            return handler

        def test_queue(handler, throttle_rate, queue_length):
            received = {"msgs": []}

            def cb(msg):
                received["msgs"].append(msg)

            handler = handler.set_throttle_rate(throttle_rate)
            handler = handler.set_queue_length(queue_length)

            throttle_rate = throttle_rate / 1000.0

            time.sleep(throttle_rate + 0.01)
            handler.last_publish = time.time()
            last_msg = time.time() + throttle_rate / 4.0

            handler.publish = cb

            xs = range(1000)
            for x in xs:
                handler.handle_message(x)

            try:
                for i in range(0, queue_length - 1):
                    time.sleep(throttle_rate - time.time() + last_msg)
                    last_msg = time.time()
                    self.assertEqual(received["msgs"], xs[len(xs) - queue_length:len(xs) - queue_length + i + 1])
            except:
                handler.finish()
                raise

            return handler

        # Test individually
        handler = subscribe.MessageHandler(None, None)
        handler = test_queue(handler, 50, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_throttle(handler, 50)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_default(handler)
        handler.finish()

        # Test combinations
        handler = subscribe.MessageHandler(None, None)
        handler = test_queue(handler, 50, 10)
        handler = test_throttle(handler, 50)
        handler = test_default(handler)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_queue(handler, 50, 10)
        handler = test_default(handler)
        handler = test_throttle(handler, 50)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_throttle(handler, 50)
        handler = test_queue(handler, 50, 10)
        handler = test_default(handler)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_throttle(handler, 50)
        handler = test_default(handler)
        handler = test_queue(handler, 50, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_default(handler)
        handler = test_throttle(handler, 50)
        handler = test_queue(handler, 50, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_default(handler)
        handler = test_queue(handler, 50, 10)
        handler = test_throttle(handler, 50)
        handler.finish()

        # Test duplicates
        handler = subscribe.MessageHandler(None, None)
        handler = test_queue(handler, 50, 10)
        handler = test_queue(handler, 100, 10)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_throttle(handler, 50)
        handler = test_throttle(handler, 100)
        handler.finish()

        handler = subscribe.MessageHandler(None, None)
        handler = test_default(handler)
        handler = test_default(handler)
        handler.finish()


#        handler = test_throttle(handler, 50)
#        handler = test_default(handler)
#        handler = test_throttle(handler, 50)
#        handler = test_default(handler)
#        handler = test_throttle(handler, 50)


PKG = 'rosbridge_library'
NAME = 'test_message_handlers'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMessageHandlers)
