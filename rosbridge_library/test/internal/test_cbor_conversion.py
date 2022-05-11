#!/usr/bin/env python3
import struct
import unittest

from rosbridge_library.internal.cbor_conversion import (
    TAGGED_ARRAY_FORMATS,
    extract_cbor_values,
)

try:
    from cbor import Tag
except ImportError:
    from rosbridge_library.util.cbor import Tag

from example_interfaces.msg import (
    Bool,
    Byte,
    ByteMultiArray,
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
    Char,
    String,
    WString,
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

    def test_wstring(self):
        msg = WString(data="foo")
        extracted = extract_cbor_values(msg)

        self.assertEqual(extracted["data"], msg.data)
        self.assertEqual(type(extracted["data"]), str)

    def test_bool(self):
        for val in [True, False]:
            msg = Bool(data=val)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted["data"], msg.data, f"val={val}")
            self.assertEqual(type(extracted["data"]), bool, f"val={val}")

    def test_byte(self):
        msg = Byte(data=b'\x01')
        extracted = extract_cbor_values(msg)

        self.assertEqual(extracted["data"], msg.data)
        self.assertEqual(type(extracted["data"]), bytes)

    def test_char(self):
        msg = Char(data=1)
        extracted = extract_cbor_values(msg)

        self.assertEqual(extracted["data"], msg.data)
        self.assertEqual(type(extracted["data"]), int)

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

    def test_byte_array(self):
        msg = ByteMultiArray(data=[b'\x00', b'\x01', b'\x02'])
        extracted = extract_cbor_values(msg)

        data = extracted["data"]
        self.assertEqual(type(data), list)
        for i, val in enumerate(msg.data):
            self.assertEqual(data[i], val)

    def test_byte_stream(self):
        msg = UInt8MultiArray(data=[0, 1, 2])
        extracted = extract_cbor_values(msg)

        data = extracted["data"]
        self.assertEqual(type(data), bytes)
        for i, val in enumerate(msg.data):
            self.assertEqual(data[i], val)

    def test_numeric_array(self):
        for msg_type in [
            Int8MultiArray,
            Int16MultiArray,
            Int32MultiArray,
            Int64MultiArray,
            UInt16MultiArray,
            UInt32MultiArray,
            UInt64MultiArray,
            Float32MultiArray,
            Float64MultiArray,
        ]:
            msg = msg_type(data=[0, 1, 2])
            extracted = extract_cbor_values(msg)

            tag = extracted["data"]
            self.assertEqual(type(tag), Tag, f"type={msg_type}")
            self.assertEqual(type(tag.value), bytes, f"type={msg_type}")

            # This is as consistent as the message defs..
            array_type = msg._fields_and_field_types['data']

            expected_tag = TAGGED_ARRAY_FORMATS[array_type][0]
            self.assertEqual(tag.tag, expected_tag, f"type={msg_type}")

            fmt = TAGGED_ARRAY_FORMATS[array_type][1]
            fmt_to_length = fmt.format(len(msg.data))
            unpacked = list(struct.unpack(fmt_to_length, tag.value))

            self.assertEqual(unpacked, list(msg.data), f"type={msg_type}")

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
