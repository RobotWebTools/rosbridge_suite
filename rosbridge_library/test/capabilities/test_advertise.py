#!/usr/bin/env python
import time
import unittest
from json import dumps, loads
from threading import Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)
from rosbridge_library.internal.publishers import manager
from rosbridge_library.protocol import Protocol
from rosbridge_library.util.ros import is_topic_published


class TestAdvertise(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_advertise")
        self.executor.add_node(self.node)
        self.exec_thread = Thread(target=self.executor.spin)
        self.exec_thread.start()

        manager.unregister_timeout = 1.0

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def test_missing_arguments(self):
        proto = Protocol("hello", self.node)
        adv = Advertise(proto)
        msg = {"op": "advertise"}
        self.assertRaises(MissingArgumentException, adv.advertise, loads(dumps(msg)))

        msg = {"op": "advertise", "topic": "/jon"}
        self.assertRaises(MissingArgumentException, adv.advertise, loads(dumps(msg)))

        msg = {"op": "advertise", "type": "std_msgs/String"}
        self.assertRaises(MissingArgumentException, adv.advertise, loads(dumps(msg)))

    def test_invalid_arguments(self):
        proto = Protocol("hello", self.node)
        adv = Advertise(proto)

        msg = {"op": "advertise", "topic": 3, "type": "std_msgs/String"}
        self.assertRaises(InvalidArgumentException, adv.advertise, loads(dumps(msg)))

        msg = {"op": "advertise", "topic": "/jon", "type": 3}
        self.assertRaises(InvalidArgumentException, adv.advertise, loads(dumps(msg)))

    def test_invalid_msg_typestrings(self):
        invalid = [
            "",
            "/",
            "//",
            "///",
            "////",
            "/////",
            "bad",
            "stillbad",
            "better/",
            "better//",
            "better///",
            "/better",
            "//better",
            "///better",
            r"this\isbad",
            "\\",
        ]

        proto = Protocol("hello", self.node)
        adv = Advertise(proto)

        for invalid_type in invalid:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_typestrings",
                "type": invalid_type,
            }
            self.assertRaises(
                ros_loader.InvalidTypeStringException, adv.advertise, loads(dumps(msg))
            )

    def test_invalid_msg_package(self):
        nonexistent = [
            "roslib/Time",
            "roslib/Duration",
            "roslib/Header",
            "std_srvs/ConflictedMsg",
            "topic_tools/MessageMessage",
            "wangle_msgs/Jam",
            "whistleblower_msgs/Document",
            "coercion_msgs/Bribe",
            "airconditioning_msgs/Cold",
            "pr2thoughts_msgs/Escape",
        ]

        proto = Protocol("hello", self.node)
        adv = Advertise(proto)

        for invalid_type in nonexistent:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_package",
                "type": invalid_type,
            }
            self.assertRaises(ros_loader.InvalidModuleException, adv.advertise, loads(dumps(msg)))

    def test_invalid_msg_classes(self):
        nonexistent = [
            "builtin_interfaces/SpaceTime",
            "std_msgs/Spool",
            "geometry_msgs/Tetrahedron",
            "sensor_msgs/TelepathyUnit",
        ]

        proto = Protocol("hello", self.node)
        adv = Advertise(proto)

        for invalid_type in nonexistent:
            msg = {
                "op": "advertise",
                "topic": "/test_invalid_msg_classes",
                "type": invalid_type,
            }
            self.assertRaises(ros_loader.InvalidClassException, adv.advertise, loads(dumps(msg)))

    def test_valid_msg_classes(self):
        assortedmsgs = [
            "geometry_msgs/Pose",
            "action_msgs/GoalStatus",
            "geometry_msgs/WrenchStamped",
            "stereo_msgs/DisparityImage",
            "nav_msgs/OccupancyGrid",
            "geometry_msgs/Point32",
            "std_msgs/String",
            "trajectory_msgs/JointTrajectoryPoint",
            "diagnostic_msgs/KeyValue",
            "visualization_msgs/InteractiveMarkerUpdate",
            "nav_msgs/GridCells",
            "sensor_msgs/PointCloud2",
        ]

        proto = Protocol("hello", self.node)
        adv = Advertise(proto)

        for valid_type in assortedmsgs:
            msg = {"op": "advertise", "topic": "/" + valid_type, "type": valid_type}
            adv.advertise(loads(dumps(msg)))
            adv.unadvertise(loads(dumps(msg)))

    def test_do_advertise(self):
        proto = Protocol("hello", self.node)
        adv = Advertise(proto)
        topic = "/test_do_advertise"
        type = "std_msgs/String"

        msg = {"op": "advertise", "topic": topic, "type": type}
        adv.advertise(loads(dumps(msg)))
        self.assertTrue(is_topic_published(self.node, topic))
        adv.unadvertise(loads(dumps(msg)))
        self.assertTrue(is_topic_published(self.node, topic))
        time.sleep(manager.unregister_timeout + 1.0)
        self.assertFalse(is_topic_published(self.node, topic))
