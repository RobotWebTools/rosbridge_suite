#!/usr/bin/env python3
import struct
import unittest
from array import array

from rosbridge_library.internal.cbor_conversion import (
    TAGGED_ARRAY_FORMATS,
    extract_cbor_values,
)

try:
    from cbor import Tag
except ImportError:
    from rosbridge_library.util.cbor import Tag

from builtin_interfaces.msg import Duration, Time
from std_msgs.msg import (
    Bool,
    Float32,
    Float32MultiArray,
    Float64,
    Float64MultiArray,
    Int8,
    Int8MultiArray,
    Int16,
    Int16MultiArray,
    Int32,
    Int32MultiArray,
    Int64,
    Int64MultiArray,
    MultiArrayDimension,
    MultiArrayLayout,
    String,
    UInt8,
    UInt8MultiArray,
    UInt16,
    UInt16MultiArray,
    UInt32,
    UInt32MultiArray,
    UInt64,
    UInt64MultiArray,
)


class TestCBORConversion(unittest.TestCase):
    def test_string(self):
        msg = String(data="foo")
        extracted = extract_cbor_values(msg)

        self.assertEqual(extracted["data"], msg.data)
        self.assertEqual(type(extracted["data"]), str)

    def test_bool(self):
        for val in [True, False]:
            msg = Bool(data=val)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted["data"], msg.data, f"val={val}")
            self.assertEqual(type(extracted["data"]), bool, f"val={val}")

    def test_numbers(self):
        for msg_type in [Int8, Int16, Int32, Int64]:
            msg = msg_type(data=-5)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted["data"], msg.data, f"type={msg_type}")
            self.assertEqual(type(extracted["data"]), int, f"type={msg_type}")

        for msg_type in [UInt8, UInt16, UInt32, UInt64]:
            msg = msg_type(data=5)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted["data"], msg.data, f"type={msg_type}")
            self.assertEqual(type(extracted["data"]), int, f"type={msg_type}")

        for msg_type in [Float32, Float64]:
            msg = msg_type(data=2.3)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted["data"], msg.data, f"type={msg_type}")
            self.assertEqual(type(extracted["data"]), float, f"type={msg_type}")

    def test_time(self):
        for msg_type in [Time, Duration]:
            msg = msg_type()
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted["sec"], msg.sec, f"type={msg_type}")
            self.assertEqual(extracted["nanosec"], msg.nanosec, f"type={msg_type}")
            self.assertEqual(type(extracted["sec"]), int, f"type={msg_type}")
            self.assertEqual(type(extracted["nanosec"]), int, f"type={msg_type}")

    def test_byte_array(self):
        msg = UInt8MultiArray(data=[0, 1, 2])
        extracted = extract_cbor_values(msg)

        data = extracted["data"]
        self.assertEqual(type(data), bytes)
        for i, val in enumerate(msg.data):
            self.assertEqual(data[i], val)

    def test_integer_array(self):
        for msg_type in [
            Int8MultiArray,
            Int16MultiArray,
            Int32MultiArray,
            Int64MultiArray,
            UInt16MultiArray,
            UInt32MultiArray,
            UInt64MultiArray,
        ]:
            msg = msg_type(data=[0, 1, 2])
            extracted = extract_cbor_values(msg)

            tag = extracted["data"]
            self.assertEqual(type(tag), Tag, f"type={msg_type}")
            self.assertEqual(type(tag.value), bytes, f"type={msg_type}")

            # This is as consistent as the message definitions.
            array_type = msg.get_fields_and_field_types()["data"]

            expected_tag = TAGGED_ARRAY_FORMATS[array_type][0]
            self.assertEqual(tag.tag, expected_tag, f"type={msg_type}")

            fmt = TAGGED_ARRAY_FORMATS[array_type][1]
            fmt_to_length = fmt.format(len(msg.data))
            unpacked = list(struct.unpack(fmt_to_length, tag.value))

            self.assertEqual(array("b", unpacked), msg.data, f"type={msg_type}")

    def test_float_array(self):
        for msg_type in [
            Float32MultiArray,
            Float64MultiArray,
        ]:
            msg = msg_type(data=[0, 1, 2])
            extracted = extract_cbor_values(msg)

            tag = extracted["data"]
            self.assertEqual(type(tag), Tag, f"type={msg_type}")
            self.assertEqual(type(tag.value), bytes, f"type={msg_type}")

            # This is as consistent as the message definitions.
            array_type = msg.get_fields_and_field_types()["data"]

            expected_tag = TAGGED_ARRAY_FORMATS[array_type][0]
            self.assertEqual(tag.tag, expected_tag, f"type={msg_type}")

            fmt = TAGGED_ARRAY_FORMATS[array_type][1]
            fmt_to_length = fmt.format(len(msg.data))
            unpacked = list(struct.unpack(fmt_to_length, tag.value))

            self.assertEqual(array("f", unpacked), msg.data, f"type={msg_type}")

    def test_nested_messages(self):
        msg = UInt8MultiArray(
            layout=MultiArrayLayout(
                data_offset=5,
                dim=[
                    MultiArrayDimension(
                        label="foo",
                        size=4,
                        stride=4,
                    ),
                    MultiArrayDimension(
                        label="bar",
                        size=8,
                        stride=8,
                    ),
                ],
            )
        )
        extracted = extract_cbor_values(msg)

        ex_layout = extracted["layout"]
        self.assertEqual(type(ex_layout), dict)
        self.assertEqual(ex_layout["data_offset"], msg.layout.data_offset)
        self.assertEqual(len(ex_layout["dim"]), len(msg.layout.dim))
        for i, val in enumerate(msg.layout.dim):
            self.assertEqual(ex_layout["dim"][i]["label"], val.label)
            self.assertEqual(ex_layout["dim"][i]["size"], val.size)
            self.assertEqual(ex_layout["dim"][i]["stride"], val.stride)

    def test_unicode_keys(self):
        msg = String(data="foo")
        extracted = extract_cbor_values(msg)

        keys = extracted.keys()
        for key in keys:
            self.assertEqual(type(key), str)
