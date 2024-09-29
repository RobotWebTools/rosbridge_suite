#!/usr/bin/env python3
import unittest
from base64 import standard_b64encode
from json import dumps, loads

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal import ros_loader


class TestMessageConversion(unittest.TestCase):
    def validate_instance(self, inst1):
        """Serializes and deserializes the inst to typecheck and ensure that
        instances are correct"""
        inst2 = deserialize_message(serialize_message(inst1), type(inst1))
        self.assertEqual(inst1, inst2)

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
        else:
            for x in msg1:
                self.assertTrue(x in msg2)
            for x in msg2:
                self.assertTrue(x in msg1)
            for x in msg1:
                self.msgs_equal(msg1[x], msg2[x])

    def do_primitive_test(self, data_value, msgtype):
        for msg in [{"data": data_value}, loads(dumps({"data": data_value}))]:
            inst = ros_loader.get_message_instance(msgtype)
            c.populate_instance(msg, inst)
            self.assertEqual(inst.data, data_value)
            self.validate_instance(inst)
            extracted = c.extract_values(inst)
            for msg2 in [extracted, loads(dumps(extracted))]:
                self.msgs_equal(msg, msg2)
                self.assertEqual(msg["data"], msg2["data"])
                self.assertEqual(msg2["data"], inst.data)

    def do_byte_test(self, data_value, msgtype):
        for msg in [{"data": data_value}]:
            inst = ros_loader.get_message_instance(msgtype)
            c.populate_instance(msg, inst)
            self.assertEqual(inst.data, bytes([data_value]))
            self.validate_instance(inst)
            extracted = c.extract_values(inst)
            for msg2 in [extracted, loads(dumps(extracted))]:
                self.assertEqual(msg["data"], msg2["data"])
                self.assertEqual(bytes([msg2["data"]]), inst.data)

    def do_test(self, orig_msg, msgtype):
        for msg in [orig_msg, loads(dumps(orig_msg))]:
            inst = ros_loader.get_message_instance(msgtype)
            c.populate_instance(msg, inst)
            self.validate_instance(inst)
            extracted = c.extract_values(inst)
            for msg2 in [extracted, loads(dumps(extracted))]:
                self.msgs_equal(msg, msg2)

    def test_int_primitives(self):
        # Test raw primitives
        for msg in range(-100, 100):
            for rostype in ["int8", "int16", "int32", "int64"]:
                self.assertEqual(c._to_primitive_inst(msg, rostype, rostype, []), msg)
                self.assertEqual(c._to_inst(msg, rostype, rostype), msg)
        # Test raw primitives
        for msg in range(0, 200):
            for rostype in ["uint8", "uint16", "uint32", "uint64"]:
                self.assertEqual(c._to_primitive_inst(msg, rostype, rostype, []), msg)
                self.assertEqual(c._to_inst(msg, rostype, rostype), msg)

    def test_byte_primitives(self):
        # Test raw primitives
        for msg in range(0, 200):
            for rostype in ["octet"]:
                self.assertEqual(c._to_primitive_inst(msg, rostype, rostype, []), bytes([msg]))
                self.assertEqual(c._to_inst(msg, rostype, rostype), bytes([msg]))

    def test_bool_primitives(self):
        self.assertTrue(c._to_primitive_inst(True, "bool", "bool", []))
        self.assertTrue(c._to_inst(True, "bool", "bool"))
        self.assertFalse(c._to_primitive_inst(False, "bool", "bool", []))
        self.assertFalse(c._to_inst(False, "bool", "bool"))

    def test_float_primitives(self):
        for msg in [0.12341234 + i for i in range(-100, 100)]:
            for rostype in ["float32", "float64"]:
                self.assertEqual(c._to_primitive_inst(msg, rostype, rostype, []), msg)
                self.assertEqual(c._to_inst(msg, rostype, rostype), msg)
                c._to_inst(msg, rostype, rostype)

    def test_float_special_cases(self):
        for msg in [1e9999999, -1e9999999, float("nan")]:
            for rostype in ["float32", "float64"]:
                self.assertEqual(c._from_inst(msg, rostype), None)
                self.assertEqual(dumps({"data": c._from_inst(msg, rostype)}), '{"data": null}')

    def test_signed_int_base_msgs(self):
        int8s = range(-127, 128)
        for int8 in int8s:
            self.do_primitive_test(int8, "std_msgs/Int8")
            self.do_primitive_test(int8, "std_msgs/Int16")
            self.do_primitive_test(int8, "std_msgs/Int32")
            self.do_primitive_test(int8, "std_msgs/Int64")

        int16s = [-32767, 32767]
        for int16 in int16s:
            self.do_primitive_test(int16, "std_msgs/Int16")
            self.do_primitive_test(int16, "std_msgs/Int32")
            self.do_primitive_test(int16, "std_msgs/Int64")
            self.assertRaises(Exception, self.do_primitive_test, int16, "std_msgs/Int8")

        int32s = [-2147483647, 2147483647]
        for int32 in int32s:
            self.do_primitive_test(int32, "std_msgs/Int32")
            self.do_primitive_test(int32, "std_msgs/Int64")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/Int8")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/Int16")

        int64s = [-9223372036854775807, 9223372036854775807]
        for int64 in int64s:
            self.do_primitive_test(int64, "std_msgs/Int64")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/Int8")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/Int16")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/Int32")

    def test_unsigned_int_base_msgs(self):
        int8s = range(0, 256)
        for int8 in int8s:
            self.do_primitive_test(int8, "std_msgs/Char")
            self.do_primitive_test(int8, "std_msgs/UInt8")
            self.do_primitive_test(int8, "std_msgs/UInt16")
            self.do_primitive_test(int8, "std_msgs/UInt32")
            self.do_primitive_test(int8, "std_msgs/UInt64")

        int16s = [32767, 32768, 65535]
        for int16 in int16s:
            self.do_primitive_test(int16, "std_msgs/UInt16")
            self.do_primitive_test(int16, "std_msgs/UInt32")
            self.do_primitive_test(int16, "std_msgs/UInt64")
            self.assertRaises(Exception, self.do_primitive_test, int16, "std_msgs/Char")
            self.assertRaises(Exception, self.do_primitive_test, int16, "std_msgs/UInt8")

        int32s = [2147483647, 2147483648, 4294967295]
        for int32 in int32s:
            self.do_primitive_test(int32, "std_msgs/UInt32")
            self.do_primitive_test(int32, "std_msgs/UInt64")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/Char")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/UInt8")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/UInt16")

        int64s = [
            4294967296,
            9223372036854775807,
            9223372036854775808,
            18446744073709551615,
        ]
        for int64 in int64s:
            self.do_primitive_test(int64, "std_msgs/UInt64")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/Char")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/UInt8")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/UInt16")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/UInt32")

    def test_byte_base_msg(self):
        int8s = range(0, 256)
        for int8 in int8s:
            self.do_byte_test(int8, "std_msgs/Byte")

    def test_bool_base_msg(self):
        self.do_primitive_test(True, "std_msgs/Bool")
        self.do_primitive_test(False, "std_msgs/Bool")

    def test_string_base_msg(self):
        for x in c.ros_primitive_types:
            self.do_primitive_test(x, "std_msgs/String")

    def test_time_msg(self):
        now_inst = c._to_inst("now", "builtin_interfaces/Time", "builtin_interfaces/Time")
        self.assertTrue("sec" in now_inst.get_fields_and_field_types())
        self.assertTrue("nanosec" in now_inst.get_fields_and_field_types())

        msg = {"sec": 3, "nanosec": 5}
        self.do_test(msg, "builtin_interfaces/Time")

        msg = {"times": [{"sec": 3, "nanosec": 5}, {"sec": 2, "nanosec": 7}]}
        self.do_test(msg, "rosbridge_test_msgs/TestTimeArray")

        # For ROS1 compatibility
        inst1 = c._to_inst(
            {"sec": 3, "nanosec": 5}, "builtin_interfaces/Time", "builtin_interfaces/Time"
        )
        inst2 = c._to_inst(
            {"secs": 3, "nsecs": 5}, "builtin_interfaces/Time", "builtin_interfaces/Time"
        )
        self.assertEqual(inst1, inst2)

    def test_duration_msg(self):
        msg = {"sec": 3, "nanosec": 5}
        self.do_test(msg, "builtin_interfaces/Duration")

        msg = {"durations": [{"sec": 3, "nanosec": 5}, {"sec": 2, "nanosec": 7}]}
        self.do_test(msg, "rosbridge_test_msgs/TestDurationArray")

    def test_header_msg(self):
        msg = {
            "stamp": {"sec": 12347, "nanosec": 322304},
            "frame_id": "2394dnfnlcx;v[p234j]",
        }
        self.do_test(msg, "std_msgs/Header")

        msg = {"header": msg}
        self.do_test(msg, "rosbridge_test_msgs/TestHeader")
        self.do_test(msg, "rosbridge_test_msgs/TestHeaderTwo")

        msg = {"header": [msg["header"], msg["header"], msg["header"]]}
        self.do_test(msg, "rosbridge_test_msgs/TestHeaderArray")

    def test_assorted_msgs(self):
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
        for rostype in assortedmsgs:
            inst = ros_loader.get_message_instance(rostype)
            msg = c.extract_values(inst)
            self.do_test(msg, rostype)
            _ = loads(dumps(msg))
            inst2 = ros_loader.get_message_instance(rostype)
            c.populate_instance(msg, inst2)
            self.assertEqual(inst, inst2)

    def test_int8array(self):
        def test_int8_msg(rostype, data):
            msg = {"data": data}
            inst = ros_loader.get_message_instance(rostype)
            c.populate_instance(msg, inst)
            self.validate_instance(inst)
            return inst.data

        for msgtype in ["TestChar", "TestUInt8"]:
            rostype = "rosbridge_test_msgs/" + msgtype

            # From List[int]
            int8s = list(range(0, 256))
            ret = test_int8_msg(rostype, int8s)
            np.testing.assert_array_equal(ret, np.array(int8s))

            # From base64 string
            b64str_int8s = standard_b64encode(bytes(int8s)).decode("ascii")
            ret = test_int8_msg(rostype, b64str_int8s)
            np.testing.assert_array_equal(ret, np.array(int8s))

        for msgtype in ["TestUInt8FixedSizeArray16"]:
            rostype = "rosbridge_test_msgs/" + msgtype

            # From List[int]
            int8s = list(range(0, 16))
            ret = test_int8_msg(rostype, int8s)
            np.testing.assert_array_equal(ret, np.array(int8s))

            # From base64 string
            b64str_int8s = standard_b64encode(bytes(int8s)).decode("ascii")
            ret = test_int8_msg(rostype, b64str_int8s)
            np.testing.assert_array_equal(ret, np.array(int8s))

    def test_float32array(self):
        def test_float32_msg(rostype, data):
            msg = {"data": data}
            inst = ros_loader.get_message_instance(rostype)
            c.populate_instance(msg, inst)
            self.validate_instance(inst)
            return inst.data

        for msgtype in ["TestFloat32Array"]:
            rostype = "rosbridge_test_msgs/" + msgtype

            # From List[float]
            floats = list(map(float, range(0, 256)))
            ret = test_float32_msg(rostype, floats)
            np.testing.assert_array_equal(ret, np.array(floats))

            # From List[int]
            ints = list(map(int, range(0, 256)))
            ret = test_float32_msg(rostype, ints)
            np.testing.assert_array_equal(ret, np.array(ints))

        for msgtype in ["TestFloat32BoundedArray"]:
            rostype = "rosbridge_test_msgs/" + msgtype

            # From List[float]
            floats = list(map(float, range(0, 16)))
            ret = test_float32_msg(rostype, floats)
            np.testing.assert_array_equal(ret, np.array(floats))

            # From List[int]
            ints = list(map(int, range(0, 16)))
            ret = test_float32_msg(rostype, ints)
            np.testing.assert_array_equal(ret, np.array(ints))

    # Test a float32 array with a length with non-numeric characters in it
    def test_float32_complexboundedarray(self):
        def test_nestedboundedarray_msg(rostype, data):
            msg = {"data": {"data": data}}
            inst = ros_loader.get_message_instance(rostype)
            c.populate_instance(msg, inst)
            self.validate_instance(inst)
            return inst.data

        for msgtype in ["TestNestedBoundedArray"]:
            rostype = "rosbridge_test_msgs/" + msgtype

            # From List[float]
            floats = list(map(float, range(0, 16)))
            ret = test_nestedboundedarray_msg(rostype, floats)

            self.assertEqual(c._from_inst(ret, rostype), {"data": floats})
