#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest
import time

from json import loads, dumps
from std_msgs.msg import String

from rosbridge_library.capabilities import subscribe
from rosbridge_library.protocol import Protocol
from rosbridge_library.protocol import InvalidArgumentException, MissingArgumentException


class TestSubscribe(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_subscribe")

    def dummy_cb(self, msg):
        pass

    def test_update_params(self):
        """ Adds a bunch of random clients to the subscription and sees whether
        the correct parameters are chosen as the min """
        client_id = "client_test_update_params"
        topic = "/test_update_params"
        msg_type = "std_msgs/String"

        subscription = subscribe.Subscription(client_id, topic, None)

        min_throttle_rate = 5
        min_queue_length = 2
        min_frag_size = 20

        for throttle_rate in range(min_throttle_rate, min_throttle_rate + 10):
            for queue_length in range(min_queue_length, min_queue_length + 10):
                for frag_size in range(min_frag_size, min_frag_size + 10):
                    sid = throttle_rate * 100 + queue_length * 10 + frag_size
                    subscription.subscribe(sid, msg_type, throttle_rate,
                                           queue_length, frag_size)

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
        except:
            subscription.unregister()
            raise

        subscription.unregister()

    def test_missing_arguments(self):
        proto = Protocol("test_missing_arguments")
        sub = subscribe.Subscribe(proto)
        msg = {"op": "subscribe"}
        self.assertRaises(MissingArgumentException, sub.subscribe, msg)

    def test_invalid_arguments(self):
        proto = Protocol("test_invalid_arguments")
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
        proto = Protocol("test_subscribe_works")
        sub = subscribe.Subscribe(proto)
        topic = "/test_subscribe_works"
        msg = String()
        msg.data = "test test_subscribe_works works"
        msg_type = "std_msgs/String"

        received = {"msg": None}

        def send(outgoing):
            received["msg"] = outgoing

        proto.send = send

        sub.subscribe(loads(dumps({"op": "subscribe", "topic": topic, "type": msg_type})))

        p = rospy.Publisher(topic, String, queue_size=5)
        time.sleep(0.25)
        p.publish(msg)

        time.sleep(0.25)
        self.assertEqual(received["msg"]["msg"]["data"], msg.data)

    def test_subscribe_ids(self):
        """ Test that when an ID is given at subscription time, messages are passed on with the given ID.
        It implies that when multiple subscrptions are done on the same topic but with different IDs the
        related messages should be all independently passed on with their own ID.
        To make the test more complete, we add one subscription with no ID and make sure that the messages
        get also passed on, with no "id" field.
        """
        import uuid
        ids = [str(uuid.uuid4()) for i in range(10)]
        ids.append(None) # testing for missing ID

        def subscription_func(make_subscription):
            for id in ids:
                make_subscription(id)

        def test(source_message, outgoing_messages):
            for message in outgoing_messages:
                self.assertEqual(message["msg"]["data"], source_message.data)
                if "id" not in message:
                    message["id"] = None # testing for missing ID
                self.assertIn(message["id"], ids)
                ids.remove(message["id"])
            self.assertFalse(ids, "The id list is not empty, which means that some"
                                "of the subscribers did not get a message.")
        
        def unsubscription_func(make_unsubscription):
            pass
        
        self.subscription_test("subscribe_ids", subscription_func, unsubscription_func, test)
    
    def test_subscribe_unsubscribe_ids(self):
        """ This test ensures that unsubscriptions with IDs are properly handled.
        Namely, multiple subscriptions are done on a same topic but with different IDs. Should one of the subscription
        be cancelled, the others shall not be affected and the messages shall still be passed on with their respective
        IDs.
        """
        import uuid
        ids = [str(uuid.uuid4()) for i in range(2)]

        def subscription_func(make_subscription):
            for id in ids:
                make_subscription(id)
        
        def unsubscription_func(make_unsubscription):
            make_unsubscription(ids[0])

        def test(source_message, outgoing_messages):
            self.assertEqual(len(outgoing_messages), 1, "Only one message must have been forwarded, as one out of two"
                                                        "subscriptions has been cancelled.")
            message = outgoing_messages[0]
            self.assertEqual(message["msg"]["data"], source_message.data)
            self.assertEqual(message["id"], ids[1])
        
        self.subscription_test("subscribe_ids", subscription_func, unsubscription_func, test)

    def subscription_test(self, test_name, subscription_func, unsubscription_func, test):
        """ Helper function used for the two above tests.
        It gathers all their boilerplate code.
        """
        proto = Protocol(test_name)
        topic = "/" + test_name + "_works"
        
        # Tricks to capture messages on the subscribed topic
        outgoing_messages = list()
        from copy import deepcopy
        def send(outgoing):
            outgoing_messages.append(deepcopy(outgoing))
        proto.send = send

        # Prepare subscriptions
        sub = subscribe.Subscribe(proto)
        def make_subscription(id=None):
            msg_type = "std_msgs/String"
            msg = {"op": "subscribe", "topic": topic, "type": msg_type}
            if id is not None:
                msg["id"] = id
            sub.subscribe(loads(dumps(msg)))

        def make_unsubscription(id=None):
            msg = {"op": "unsubscribe", "topic": topic}
            if id is not None:
                msg["id"] = id
            sub.unsubscribe(loads(dumps(msg)))

        # Make subscriptions
        subscription_func(make_subscription)

        # Make unsubscriptions
        unsubscription_func(make_unsubscription)

        # Publish a message on the same topic in ROS (through the bridge)
        msg = String()
        msg.data = "test that " + test_name + " works"
        p = rospy.Publisher(topic, String, queue_size=5)
        time.sleep(0.25)
        p.publish(msg)

        # Test the values
        time.sleep(2.5) # Let the messages propagate to and from ROS.

        test(msg, outgoing_messages)
        


PKG = 'rosbridge_library'
NAME = 'test_subscribe'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestSubscribe)

