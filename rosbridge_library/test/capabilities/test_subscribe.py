#!/usr/bin/env python
import time
import unittest
from json import dumps, loads
from threading import Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.capabilities import subscribe
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.protocol import Protocol
from std_msgs.msg import String


class TestSubscribe(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_subscribe")
        self.executor.add_node(self.node)

        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def dummy_cb(self, msg):
        pass

    def test_update_params(self):
        """Adds a bunch of random clients to the subscription and sees whether
        the correct parameters are chosen as the min"""
        client_id = "client_test_update_params"
        topic = "/test_update_params"
        msg_type = "std_msgs/String"

        subscription = subscribe.Subscription(client_id, topic, None, self.node)

        min_throttle_rate = 5
        min_queue_length = 2
        min_frag_size = 20

        for throttle_rate in range(min_throttle_rate, min_throttle_rate + 10):
            for queue_length in range(min_queue_length, min_queue_length + 10):
                for frag_size in range(min_frag_size, min_frag_size + 10):
                    sid = throttle_rate * 100 + queue_length * 10 + frag_size
                    subscription.subscribe(sid, msg_type, throttle_rate, queue_length, frag_size)

        subscription.update_params()

        try:
            self.assertEqual(subscription.throttle_rate, min_throttle_rate)
            self.assertEqual(subscription.queue_length, min_queue_length)
            self.assertEqual(subscription.fragment_size, min_frag_size)
            self.assertEqual(subscription.compression, "none")

            list(subscription.clients.values())[0]["compression"] = "png"

            subscription.update_params()

            self.assertEqual(subscription.throttle_rate, min_throttle_rate)
            self.assertEqual(subscription.queue_length, min_queue_length)
            self.assertEqual(subscription.fragment_size, min_frag_size)
            self.assertEqual(subscription.compression, "png")
        finally:
            subscription.unregister()

    def test_missing_arguments(self):
        proto = Protocol("test_missing_arguments", self.node)
        sub = subscribe.Subscribe(proto)
        msg = {"op": "subscribe"}
        self.assertRaises(MissingArgumentException, sub.subscribe, msg)

    def test_invalid_arguments(self):
        proto = Protocol("test_invalid_arguments", self.node)
        sub = subscribe.Subscribe(proto)

        msg = {"op": "subscribe", "topic": 3}
        self.assertRaises(InvalidArgumentException, sub.subscribe, msg)

        msg = {"op": "subscribe", "topic": "/jon", "type": 3}
        self.assertRaises(InvalidArgumentException, sub.subscribe, msg)

        msg = {"op": "subscribe", "topic": "/jon", "throttle_rate": "fast"}
        self.assertRaises(InvalidArgumentException, sub.subscribe, msg)

        msg = {"op": "subscribe", "topic": "/jon", "fragment_size": "five cubits"}
        self.assertRaises(InvalidArgumentException, sub.subscribe, msg)

        msg = {"op": "subscribe", "topic": "/jon", "queue_length": "long"}
        self.assertRaises(InvalidArgumentException, sub.subscribe, msg)

        msg = {"op": "subscribe", "topic": "/jon", "compression": 9000}
        self.assertRaises(InvalidArgumentException, sub.subscribe, msg)

    def test_subscribe_works(self):
        proto = Protocol("test_subscribe_works", self.node)
        sub = subscribe.Subscribe(proto)
        topic = "/test_subscribe_works"
        msg = String()
        msg.data = "test test_subscribe_works works"
        msg_type = "std_msgs/String"

        received = {"msg": None}

        def send(outgoing, **kwargs):
            received["msg"] = outgoing

        proto.send = send

        sub.subscribe(loads(dumps({"op": "subscribe", "topic": topic, "type": msg_type})))

        publisher_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pub = self.node.create_publisher(String, topic, publisher_qos)
        pub.publish(msg)
        time.sleep(0.1)
        self.assertEqual(received["msg"]["msg"]["data"], msg.data)
