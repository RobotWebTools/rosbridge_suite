#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest
from rosgraph import Master

from time import sleep, time

from rosbridge_library.internal.subscribers import *
from rosbridge_library.internal.topics import *
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import *
from std_msgs.msg import String, Int32


class TestMultiSubscriber(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_multi_subscriber")

    def is_topic_published(self, topicname):
        return topicname in dict(rospy.get_published_topics()).keys()

    def is_topic_subscribed(self, topicname):
        return topicname in dict(Master("test_multi_subscriber").getSystemState()[1])

    def test_register_multisubscriber(self):
        """ Register a subscriber on a clean topic with a good msg type """
        topic = "/test_register_multisubscriber"
        msg_type = "std_msgs/String"

        self.assertFalse(self.is_topic_subscribed(topic))
        MultiSubscriber(topic, msg_type)
        self.assertTrue(self.is_topic_subscribed(topic))

    def test_unregister_multisubscriber(self):
        """ Register and unregister a subscriber on a clean topic with a good msg type """
        topic = "/test_unregister_multisubscriber"
        msg_type = "std_msgs/String"

        self.assertFalse(self.is_topic_subscribed(topic))
        multi = MultiSubscriber(topic, msg_type)
        self.assertTrue(self.is_topic_subscribed(topic))
        multi.unregister()
        self.assertFalse(self.is_topic_subscribed(topic))

    def test_verify_type(self):
        topic = "/test_verify_type"
        msg_type = "std_msgs/String"
        othertypes = ["geometry_msgs/Pose", "actionlib_msgs/GoalStatus",
        "geometry_msgs/WrenchStamped", "stereo_msgs/DisparityImage",
        "nav_msgs/OccupancyGrid", "geometry_msgs/Point32",
        "trajectory_msgs/JointTrajectoryPoint", "diagnostic_msgs/KeyValue",
        "visualization_msgs/InteractiveMarkerUpdate", "nav_msgs/GridCells",
        "sensor_msgs/PointCloud2"]

        s = MultiSubscriber(topic, msg_type)
        s.verify_type(msg_type)
        for othertype in othertypes:
            self.assertRaises(TypeConflictException, s.verify_type, othertype)

    def test_subscribe_unsubscribe(self):
        topic = "/test_subscribe_unsubscribe"
        msg_type = "std_msgs/String"
        client = "client_test_subscribe_unsubscribe"

        self.assertFalse(self.is_topic_subscribed(topic))
        multi = MultiSubscriber(topic, msg_type)
        self.assertTrue(self.is_topic_subscribed(topic))
        self.assertFalse(multi.has_subscribers())

        multi.subscribe(client, None)
        self.assertTrue(multi.has_subscribers())

        multi.unsubscribe(client)
        self.assertFalse(multi.has_subscribers())

        multi.unregister()
        self.assertFalse(self.is_topic_subscribed(topic))

    def test_subscribe_receive_json(self):
        topic = "/test_subscribe_receive_json"
        msg_type = "std_msgs/String"
        client = "client_test_subscribe_receive_json"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        pub = rospy.Publisher(topic, String)
        multi = MultiSubscriber(topic, msg_type)

        received = {"msg": None}

        def cb(msg):
            received["msg"] = msg

        multi.subscribe(client, cb)
        sleep(0.5)
        pub.publish(msg)
        sleep(0.5)
        self.assertEqual(msg.data, received["msg"]["data"])

    def test_subscribe_receive_json_multiple(self):
        topic = "/test_subscribe_receive_json_multiple"
        msg_type = "std_msgs/Int32"
        client = "client_test_subscribe_receive_json_multiple"

        numbers = list(range(100))

        pub = rospy.Publisher(topic, Int32)
        multi = MultiSubscriber(topic, msg_type)

        received = {"msgs": []}

        def cb(msg):
            received["msgs"].append(msg["data"])

        multi.subscribe(client, cb)
        sleep(0.5)
        for x in numbers:
            msg = Int32()
            msg.data = x
            pub.publish(msg)
        sleep(0.5)
        self.assertEqual(numbers, received["msgs"])

    def test_unsubscribe_does_not_receive_further_msgs(self):
        topic = "/test_unsubscribe_does_not_receive_further_msgs"
        msg_type = "std_msgs/String"
        client = "client_test_unsubscribe_does_not_receive_further_msgs"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        pub = rospy.Publisher(topic, String)
        multi = MultiSubscriber(topic, msg_type)

        received = {"count": 0}

        def cb(msg):
            received["count"] = received["count"] + 1

        multi.subscribe(client, cb)
        sleep(0.5)
        pub.publish(msg)
        sleep(0.5)
        self.assertEqual(received["count"], 1)
        multi.unsubscribe(client)
        sleep(0.5)
        pub.publish(msg)
        sleep(0.5)
        self.assertEqual(received["count"], 1)

    def test_multiple_subscribers(self):
        topic = "/test_subscribe_receive_json"
        msg_type = "std_msgs/String"
        client1 = "client_test_subscribe_receive_json_1"
        client2 = "client_test_subscribe_receive_json_2"

        msg = String()
        msg.data = "dsajfadsufasdjf"

        pub = rospy.Publisher(topic, String)
        multi = MultiSubscriber(topic, msg_type)

        received = {"msg1": None, "msg2": None}

        def cb1(msg):
            received["msg1"] = msg

        def cb2(msg):
            received["msg2"] = msg

        multi.subscribe(client1, cb1)
        multi.subscribe(client2, cb2)
        sleep(0.5)
        pub.publish(msg)
        sleep(0.5)
        self.assertEqual(msg.data, received["msg1"]["data"])
        self.assertEqual(msg.data, received["msg2"]["data"])


PKG = 'rosbridge_library'
NAME = 'test_multi_subscriber'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMultiSubscriber)

