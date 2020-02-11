#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import rostest
import unittest
from json import loads, dumps

try:
    from cStringIO import StringIO  # Python 2.x
except ImportError:
    from io import BytesIO as StringIO  # Python 3.x

from rosbridge_library.internal import message_conversion as c
from rosbridge_library.internal import ros_loader
from base64 import standard_b64encode, standard_b64decode

if sys.version_info >= (3, 0):
    string_types = (str,)
else:
    string_types = (str, unicode)


class TestMessageConversion(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_message_conversion")

    def validate_instance(self, inst1):
        """ Serializes and deserializes the inst to typecheck and ensure that
        instances are correct """
        inst1._check_types()
        buff = StringIO()
        inst1.serialize(buff)
        inst2 = type(inst1)()
        inst2.deserialize(buff.getvalue())
        self.assertEqual(inst1, inst2)
        inst2._check_types()

    def msgs_equal(self, msg1, msg2):
        if type(msg1) in string_types and type(msg2) in string_types:
            pass
        else:
            self.assertEqual(type(msg1), type(msg2))
        if type(msg1) in c.list_types:
            for x, y in zip(msg1, msg2):
                self.msgs_equal(x, y)
        elif type(msg1) in c.primitive_types or type(msg1) in c.string_types:
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
        for msg in [1e9999999, -1e9999999, float('nan')]:
            for rostype in ["float32", "float64"]:
                self.assertEqual(c._from_inst(msg, rostype), None)
                self.assertEqual(dumps({"data":c._from_inst(msg, rostype)}), "{\"data\": null}")

    def test_signed_int_base_msgs(self):
        int8s = range(-127, 128)
        for int8 in int8s:
            self.do_primitive_test(int8, "std_msgs/Byte")
            self.do_primitive_test(int8, "std_msgs/Int8")
            self.do_primitive_test(int8, "std_msgs/Int16")
            self.do_primitive_test(int8, "std_msgs/Int32")
            self.do_primitive_test(int8, "std_msgs/Int64")

        int16s = [-32767, 32767]
        for int16 in int16s:
            self.do_primitive_test(int16, "std_msgs/Int16")
            self.do_primitive_test(int16, "std_msgs/Int32")
            self.do_primitive_test(int16, "std_msgs/Int64")
            self.assertRaises(Exception, self.do_primitive_test, int16, "std_msgs/Byte")
            self.assertRaises(Exception, self.do_primitive_test, int16, "std_msgs/Int8")

        int32s = [-2147483647, 2147483647]
        for int32 in int32s:
            self.do_primitive_test(int32, "std_msgs/Int32")
            self.do_primitive_test(int32, "std_msgs/Int64")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/Byte")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/Int8")
            self.assertRaises(Exception, self.do_primitive_test, int32, "std_msgs/Int16")

        int64s = [-9223372036854775807, 9223372036854775807]
        for int64 in int64s:
            self.do_primitive_test(int64, "std_msgs/Int64")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/Byte")
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

        int64s = [4294967296, 9223372036854775807, 9223372036854775808,
                   18446744073709551615]
        for int64 in int64s:
            self.do_primitive_test(int64, "std_msgs/UInt64")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/Char")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/UInt8")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/UInt16")
            self.assertRaises(Exception, self.do_primitive_test, int64, "std_msgs/UInt32")

    def test_bool_base_msg(self):
        self.do_primitive_test(True, "std_msgs/Bool")
        self.do_primitive_test(False, "std_msgs/Bool")

    def test_string_base_msg(self):
        for x in c.ros_primitive_types:
            self.do_primitive_test(x, "std_msgs/String")

    def test_time_msg(self):
        msg = {"data": {"secs": 3, "nsecs": 5}}
        self.do_test(msg, "std_msgs/Time")

        msg = {"times": [{"secs": 3, "nsecs": 5}, {"secs": 2, "nsecs": 7}]}
        self.do_test(msg, "rosbridge_library/TestTimeArray")

    def test_time_msg_now(self):
        msg = {"data": "now"}
        msgtype = "std_msgs/Time"

        inst = ros_loader.get_message_instance(msgtype)
        c.populate_instance(msg, inst)
        currenttime = rospy.get_rostime()
        self.validate_instance(inst)
        extracted = c.extract_values(inst)
        print(extracted)
        self.assertIn("data", extracted)
        self.assertIn("secs", extracted["data"])
        self.assertIn("nsecs", extracted["data"])
        self.assertNotEqual(extracted["data"]["secs"], 0)
        self.assertLessEqual(extracted["data"]["secs"], currenttime.secs)
        self.assertGreaterEqual(currenttime.secs, extracted["data"]["secs"])

    def test_duration_msg(self):
        msg = {"data": {"secs": 3, "nsecs": 5}}
        self.do_test(msg, "std_msgs/Duration")

        msg = {"durations": [{"secs": 3, "nsecs": 5}, {"secs": 2, "nsecs": 7}]}
        self.do_test(msg, "rosbridge_library/TestDurationArray")

    def test_header_msg(self):
        msg = {"seq": 5, "stamp": {"secs": 12347, "nsecs": 322304}, "frame_id": "2394dnfnlcx;v[p234j]"}
        self.do_test(msg, "std_msgs/Header")

        msg = {"header": msg}
        self.do_test(msg, "rosbridge_library/TestHeader")
        self.do_test(msg, "rosbridge_library/TestHeaderTwo")

        msg = {"header": [msg["header"], msg["header"], msg["header"]]}
        msg["header"][1]["seq"] = 6
        msg["header"][2]["seq"] = 7
        self.do_test(msg, "rosbridge_library/TestHeaderArray")

    def test_assorted_msgs(self):
        assortedmsgs = ["geometry_msgs/Pose", "actionlib_msgs/GoalStatus",
        "geometry_msgs/WrenchStamped", "stereo_msgs/DisparityImage",
        "nav_msgs/OccupancyGrid", "geometry_msgs/Point32", "std_msgs/String",
        "trajectory_msgs/JointTrajectoryPoint", "diagnostic_msgs/KeyValue",
        "visualization_msgs/InteractiveMarkerUpdate", "nav_msgs/GridCells",
        "sensor_msgs/PointCloud2"]
        for rostype in assortedmsgs:
            inst = ros_loader.get_message_instance(rostype)
            msg = c.extract_values(inst)
            self.do_test(msg, rostype)
            l = loads(dumps(msg))
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
            rostype = "rosbridge_library/" + msgtype

            int8s = list(range(0, 256))
            ret = test_int8_msg(rostype, int8s)
            self.assertEqual(ret, bytes(bytearray(int8s)))

            str_int8s = bytes(bytearray(int8s))

            b64str_int8s = standard_b64encode(str_int8s).decode('ascii')
            ret = test_int8_msg(rostype, b64str_int8s)
            self.assertEqual(ret, str_int8s)

        for msgtype in ["TestUInt8FixedSizeArray16"]:
            rostype = "rosbridge_library/" + msgtype

            int8s = list(range(0, 16))
            ret = test_int8_msg(rostype, int8s)
            self.assertEqual(ret, bytes(bytearray(int8s)))

            str_int8s = bytes(bytearray(int8s))

            b64str_int8s = standard_b64encode(str_int8s).decode('ascii')
            ret = test_int8_msg(rostype, b64str_int8s)
            self.assertEqual(ret, str_int8s)


PKG = 'rosbridge_library'
NAME = 'test_message_conversion'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestMessageConversion)
