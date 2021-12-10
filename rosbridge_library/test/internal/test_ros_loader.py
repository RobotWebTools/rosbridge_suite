#!/usr/bin/env python
import unittest

from rosbridge_library.internal import ros_loader
from rosidl_runtime_py.utilities import get_message


class TestROSLoader(unittest.TestCase):
    def test_bad_msgnames(self):
        bad = [
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
        for x in bad:
            self.assertRaises(
                ros_loader.InvalidTypeStringException, ros_loader.get_message_class, x
            )
            self.assertRaises(
                ros_loader.InvalidTypeStringException, ros_loader.get_message_instance, x
            )

    def test_irregular_msgnames(self):
        irregular = [
            "std_msgs//String",
            "//std_msgs/String",
            "/std_msgs//String",
            "/std_msgs/String",
            "//std_msgs//String",
            "/std_msgs/String/",
            "//std_msgs//String//",
            "std_msgs/String/",
            "std_msgs//String//",
        ]
        for x in irregular:
            self.assertNotEqual(ros_loader.get_message_class(x), None)
            self.assertNotEqual(ros_loader.get_message_instance(x), None)

    def test_std_msgnames(self):
        stdmsgs = [
            "std_msgs/Bool",
            "std_msgs/Byte",
            "std_msgs/ByteMultiArray",
            "std_msgs/ColorRGBA",
            "std_msgs/Empty",
            "std_msgs/Float32",
            "std_msgs/Float32MultiArray",
            "std_msgs/Float64",
            "std_msgs/Header",
            "std_msgs/Int16",
            "std_msgs/Int16MultiArray",
            "std_msgs/Int32",
            "std_msgs/Int32MultiArray",
            "std_msgs/Int64",
            "std_msgs/Int64MultiArray",
            "std_msgs/Int8",
            "std_msgs/Int8MultiArray",
            "std_msgs/MultiArrayDimension",
            "std_msgs/MultiArrayLayout",
            "std_msgs/String",
            "std_msgs/UInt16",
            "std_msgs/UInt16MultiArray",
            "std_msgs/UInt32MultiArray",
            "std_msgs/UInt64MultiArray",
            "std_msgs/UInt32",
            "std_msgs/UInt64",
            "std_msgs/UInt8",
            "std_msgs/UInt8MultiArray",
        ]
        for x in stdmsgs:
            self.assertNotEqual(ros_loader.get_message_class(x), None)
            inst = ros_loader.get_message_instance(x)
            self.assertNotEqual(inst, None)
            self.assertEqual(get_message(x), type(inst))

    def test_msg_cache(self):
        stdmsgs = [
            "std_msgs/Bool",
            "std_msgs/Byte",
            "std_msgs/ByteMultiArray",
            "std_msgs/ColorRGBA",
            "std_msgs/Empty",
            "std_msgs/Float32",
            "std_msgs/Float32MultiArray",
            "std_msgs/Float64",
            "std_msgs/Header",
            "std_msgs/Int16",
            "std_msgs/Int16MultiArray",
            "std_msgs/Int32",
            "std_msgs/Int32MultiArray",
            "std_msgs/Int64",
            "std_msgs/Int64MultiArray",
            "std_msgs/Int8",
            "std_msgs/Int8MultiArray",
            "std_msgs/MultiArrayDimension",
            "std_msgs/MultiArrayLayout",
            "std_msgs/String",
            "std_msgs/UInt16",
            "std_msgs/UInt16MultiArray",
            "std_msgs/UInt32MultiArray",
            "std_msgs/UInt64MultiArray",
            "std_msgs/UInt32",
            "std_msgs/UInt64",
            "std_msgs/UInt8",
            "std_msgs/UInt8MultiArray",
        ]
        for x in stdmsgs:
            self.assertNotEqual(ros_loader.get_message_class(x), None)
            inst = ros_loader.get_message_instance(x)
            self.assertNotEqual(inst, None)
            self.assertEqual(get_message(x), type(inst))
            self.assertTrue(x in ros_loader._loaded_msgs)

    def test_assorted_msgnames(self):
        assortedmsgs = [
            "geometry_msgs/Pose",
            "actionlib_msgs/GoalStatus",
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
        for x in assortedmsgs:
            self.assertNotEqual(ros_loader.get_message_class(x), None)
            inst = ros_loader.get_message_instance(x)
            self.assertNotEqual(inst, None)
            self.assertEqual(get_message(x), type(inst))

    def test_invalid_msgnames_primitives(self):
        invalid = [
            "bool",
            "int8",
            "uint8",
            "int16",
            "uint16",
            "int32",
            "uint32",
            "int64",
            "uint64",
            "float32",
            "float64",
            "string",
            "time",
            "duration",
        ]
        for x in invalid:
            self.assertRaises(
                ros_loader.InvalidTypeStringException, ros_loader.get_message_class, x
            )
            self.assertRaises(
                ros_loader.InvalidTypeStringException,
                ros_loader.get_message_instance,
                x,
            )

    def test_nonexistent_packagenames(self):
        nonexistent = [
            "wangle_msgs/Jam",
            "whistleblower_msgs/Document",
            "coercion_msgs/Bribe",
            "airconditioning_msgs/Cold",
            "pr2thoughts_msgs/Escape",
        ]
        for x in nonexistent:
            self.assertRaises(ros_loader.InvalidModuleException, ros_loader.get_message_class, x)
            self.assertRaises(ros_loader.InvalidModuleException, ros_loader.get_message_instance, x)

    def test_packages_without_msgs(self):
        no_msgs = [
            "roslib/Time",
            "roslib/Duration",
            "roslib/Header",
            "std_srvs/ConflictedMsg",
        ]
        for x in no_msgs:
            self.assertRaises(ros_loader.InvalidModuleException, ros_loader.get_message_class, x)
            self.assertRaises(ros_loader.InvalidModuleException, ros_loader.get_message_instance, x)

    def test_nonexistent_msg_classnames(self):
        nonexistent = [
            "rcl_interfaces/Time",
            "rcl_interfaces/Duration",
            "rcl_interfaces/Header",
            "std_msgs/Spool",
            "geometry_msgs/Tetrahedron",
            "sensor_msgs/TelepathyUnit",
        ]
        for x in nonexistent:
            self.assertRaises(ros_loader.InvalidClassException, ros_loader.get_message_class, x)
            self.assertRaises(ros_loader.InvalidClassException, ros_loader.get_message_instance, x)

    def test_bad_servicenames(self):
        bad = [
            "",
            "/",
            "//",
            "///",
            "////",
            "/////",
            "bad",
            "stillbad",
            "not/better/even/still",
            "not//better//even//still",
            "not///better///even///still",
            "better/",
            "better//",
            "better///",
            "/better",
            "//better",
            "///better",
            r"this\isbad",
            "\\",
        ]
        for x in bad:
            self.assertRaises(
                ros_loader.InvalidTypeStringException, ros_loader.get_service_class, x
            )
            self.assertRaises(
                ros_loader.InvalidTypeStringException, ros_loader.get_service_request_instance, x
            )
            self.assertRaises(
                ros_loader.InvalidTypeStringException, ros_loader.get_service_response_instance, x
            )

    def test_irregular_servicenames(self):
        irregular = [
            "rcl_interfaces//GetParameters",
            "/rcl_interfaces/GetParameters/",
            "/rcl_interfaces/GetParameters",
            "//rcl_interfaces/GetParameters",
            "/rcl_interfaces//GetParameters",
            "rcl_interfaces/GetParameters//",
            "/rcl_interfaces/GetParameters//",
            "rcl_interfaces/GetParameters/",
            "rcl_interfaces//GetParameters//",
        ]
        for x in irregular:
            self.assertNotEqual(ros_loader.get_service_class(x), None)
            self.assertNotEqual(ros_loader.get_service_request_instance(x), None)
            self.assertNotEqual(ros_loader.get_service_response_instance(x), None)

    def test_common_servicenames(self):
        common = [
            "rcl_interfaces/GetParameters",
            "rcl_interfaces/SetParameters",
            "std_srvs/Empty",
            "nav_msgs/GetMap",
            "nav_msgs/GetPlan",
            "sensor_msgs/SetCameraInfo",
            "tf2_msgs/FrameGraph",
            "example_interfaces/AddTwoInts",
        ]
        for x in common:
            self.assertNotEqual(ros_loader.get_service_class(x), None)
            self.assertNotEqual(ros_loader.get_service_request_instance(x), None)
            self.assertNotEqual(ros_loader.get_service_response_instance(x), None)

    def test_srv_cache(self):
        common = [
            "rcl_interfaces/GetParameters",
            "rcl_interfaces/SetParameters",
            "std_srvs/Empty",
            "nav_msgs/GetMap",
            "nav_msgs/GetPlan",
            "sensor_msgs/SetCameraInfo",
            "tf2_msgs/FrameGraph",
            "example_interfaces/AddTwoInts",
        ]
        for x in common:
            self.assertNotEqual(ros_loader.get_service_class(x), None)
            self.assertNotEqual(ros_loader.get_service_request_instance(x), None)
            self.assertNotEqual(ros_loader.get_service_response_instance(x), None)
            self.assertTrue(x in ros_loader._loaded_srvs)

    def test_packages_without_srvs(self):
        no_msgs = ["roslib/A", "roslib/B", "roslib/C", "std_msgs/CuriousSrv"]
        for x in no_msgs:
            self.assertRaises(ros_loader.InvalidModuleException, ros_loader.get_service_class, x)
            self.assertRaises(
                ros_loader.InvalidModuleException, ros_loader.get_service_request_instance, x
            )
            self.assertRaises(
                ros_loader.InvalidModuleException, ros_loader.get_service_response_instance, x
            )

    def test_nonexistent_service_packagenames(self):
        nonexistent = [
            "butler_srvs/FetchDrink",
            "money_srvs/MoreMoney",
            "snoopdogg_srvs/SipOnGinAndJuice",
            "revenge_srvs/BackStab",
        ]
        for x in nonexistent:
            self.assertRaises(ros_loader.InvalidModuleException, ros_loader.get_service_class, x)
            self.assertRaises(
                ros_loader.InvalidModuleException, ros_loader.get_service_request_instance, x
            )
            self.assertRaises(
                ros_loader.InvalidModuleException, ros_loader.get_service_response_instance, x
            )

    def test_nonexistent_service_classnames(self):
        nonexistent = [
            "std_srvs/Reboot",
            "std_srvs/Full",
            "nav_msgs/LoseMap",
        ]
        for x in nonexistent:
            self.assertRaises(ros_loader.InvalidClassException, ros_loader.get_service_class, x)
            self.assertRaises(
                ros_loader.InvalidClassException, ros_loader.get_service_request_instance, x
            )
            self.assertRaises(
                ros_loader.InvalidClassException, ros_loader.get_service_response_instance, x
            )
