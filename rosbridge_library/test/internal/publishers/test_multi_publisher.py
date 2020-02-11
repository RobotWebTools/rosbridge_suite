#!/usr/bin/env python
import sys
import rospy
import rostest
import unittest

from time import sleep, time

from rosbridge_library.internal.publishers import *
from rosbridge_library.internal.topics import *
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import *
from std_msgs.msg import String, Int32


class TestMultiPublisher(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_multi_publisher")

    def is_topic_published(self, topicname):
        return topicname in dict(rospy.get_published_topics()).keys()

    def test_register_multipublisher(self):
        """ Register a publisher on a clean topic with a good msg type """
        topic = "/test_register_multipublisher"
        msg_type = "std_msgs/String"

        self.assertFalse(self.is_topic_published(topic))
        multipublisher = MultiPublisher(topic, msg_type)
        self.assertTrue(self.is_topic_published(topic))

    def test_unregister_multipublisher(self):
        """ Register and unregister a publisher on a clean topic with a good msg type """
        topic = "/test_unregister_multipublisher"
        msg_type = "std_msgs/String"

        self.assertFalse(self.is_topic_published(topic))
        multipublisher = MultiPublisher(topic, msg_type)
        self.assertTrue(self.is_topic_published(topic))
        multipublisher.unregister()
        self.assertFalse(self.is_topic_published(topic))

    def test_register_client(self):
        """ Adds a publisher then removes it.  """
        topic = "/test_register_client"
        msg_type = "std_msgs/String"
        client_id = "client1"

        p = MultiPublisher(topic, msg_type)
        self.assertFalse(p.has_clients())

        p.register_client(client_id)
        self.assertTrue(p.has_clients())

        p.unregister_client(client_id)
        self.assertFalse(p.has_clients())

    def test_register_multiple_clients(self):
        """ Adds multiple publishers then removes them.  """
        topic = "/test_register_multiple_clients"
        msg_type = "std_msgs/String"

        p = MultiPublisher(topic, msg_type)
        self.assertFalse(p.has_clients())

        for i in range(1000):
            p.register_client("client%d" % i)
            self.assertTrue(p.has_clients())

        for i in range(1000):
            self.assertTrue(p.has_clients())
            p.unregister_client("client%d" % i)

        self.assertFalse(p.has_clients())

    def test_verify_type(self):
        topic = "/test_verify_type"
        msg_type = "std_msgs/String"
        othertypes = ["geometry_msgs/Pose", "actionlib_msgs/GoalStatus",
        "geometry_msgs/WrenchStamped", "stereo_msgs/DisparityImage",
        "nav_msgs/OccupancyGrid", "geometry_msgs/Point32",
        "trajectory_msgs/JointTrajectoryPoint", "diagnostic_msgs/KeyValue",
        "visualization_msgs/InteractiveMarkerUpdate", "nav_msgs/GridCells",
        "sensor_msgs/PointCloud2"]

        p = MultiPublisher(topic, msg_type)
        p.verify_type(msg_type)
        for othertype in othertypes:
            self.assertRaises(TypeConflictException, p.verify_type, othertype)

    def test_publish(self):
        """ Make sure that publishing works """
        topic = "/test_publish"
        msg_type = "std_msgs/String"
        msg = {"data": "why halo thar"}

        received = {"msg": None}
        def cb(msg):
            received["msg"] = msg

        rospy.Subscriber(topic, ros_loader.get_message_class(msg_type), cb)
        p = MultiPublisher(topic, msg_type)
        p.publish(msg)

        sleep(0.5)

        self.assertEqual(received["msg"].data, msg["data"])

    def test_bad_publish(self):
        """ Make sure that bad publishing fails """
        topic = "/test_publish"
        msg_type = "std_msgs/String"
        msg = {"data": 3}

        p = MultiPublisher(topic, msg_type)
        self.assertRaises(FieldTypeMismatchException, p.publish, msg)


PKG = 'rosbridge_library'
NAME = 'test_multi_publisher'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMultiPublisher)

