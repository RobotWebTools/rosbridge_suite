#!/usr/bin/env python
import rostest
import sys
import unittest

PYTHON2 = sys.version_info < (3, 0)

import struct
from rosbridge_library.internal.cbor_conversion import extract_cbor_values, TAGGED_ARRAY_FORMATS

try:
    from cbor import Tag
except ImportError:
    from rosbridge_library.util.cbor import Tag

from std_msgs.msg import (
    Bool, String,
    Byte, Char,
    Int8, Int16, Int32, Int64,
    UInt8, UInt16, UInt32, UInt64,
    Float32, Float64,
    ByteMultiArray,
    Int8MultiArray, Int16MultiArray, Int32MultiArray, Int64MultiArray,
    UInt8MultiArray, UInt16MultiArray, UInt32MultiArray, UInt64MultiArray,
    Float32MultiArray, Float64MultiArray,
    Time, Duration,
    Empty,
    MultiArrayLayout, MultiArrayDimension,
)


class TestCBORConversion(unittest.TestCase):
    def test_string(self):
        msg = String(data="foo")
        extracted = extract_cbor_values(msg)

        self.assertEqual(extracted['data'], msg.data)
        if PYTHON2:
            self.assertEqual(type(extracted['data']), unicode)  # noqa: F821
        else:
            self.assertEqual(type(extracted['data']), str)

    def test_bool(self):
        for val in [True, False]:
            msg = Bool(data=val)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted['data'], msg.data, 'val={}'.format(val))
            self.assertEqual(type(extracted['data']), bool, 'val={}'.format(val))

    def test_numbers(self):
        for msg_type in [Int8, Int16, Int32, Int64]:
            msg = msg_type(data=-5)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted['data'], msg.data, 'type={}'.format(msg_type))
            self.assertEqual(type(extracted['data']), int, 'type={}'.format(msg_type))

        for msg_type in [UInt8, UInt16, UInt32, UInt64]:
            msg = msg_type(data=5)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted['data'], msg.data, 'type={}'.format(msg_type))
            self.assertEqual(type(extracted['data']), int, 'type={}'.format(msg_type))

        for msg_type in [Float32, Float64]:
            msg = msg_type(data=2.3)
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted['data'], msg.data, 'type={}'.format(msg_type))
            self.assertEqual(type(extracted['data']), float, 'type={}'.format(msg_type))

    def test_time(self):
        for msg_type in [Time, Duration]:
            msg = msg_type()
            extracted = extract_cbor_values(msg)

            self.assertEqual(extracted['data']['secs'], msg.data.secs, 'type={}'.format(msg_type))
            self.assertEqual(extracted['data']['nsecs'], msg.data.nsecs, 'type={}'.format(msg_type))
            self.assertEqual(type(extracted['data']['secs']), int, 'type={}'.format(msg_type))
            self.assertEqual(type(extracted['data']['nsecs']), int, 'type={}'.format(msg_type))

    def test_byte_array(self):
        msg = UInt8MultiArray(data=[0, 1, 2])
        extracted = extract_cbor_values(msg)

        data = extracted['data']
        self.assertEqual(type(data), bytes)
        for i, val in enumerate(msg.data):
            if PYTHON2:
                self.assertEqual(ord(data[i]), val)
            else:
                self.assertEqual(data[i], val)

    def test_numeric_array(self):
        for msg_type in [Int8MultiArray, Int16MultiArray, Int32MultiArray, Int64MultiArray,
                         UInt16MultiArray, UInt32MultiArray, UInt64MultiArray,
                         Float32MultiArray, Float64MultiArray]:
            msg = msg_type(data=[0, 1, 2])
            extracted = extract_cbor_values(msg)

            tag = extracted['data']
            self.assertEqual(type(tag), Tag, 'type={}'.format(msg_type))
            self.assertEqual(type(tag.value), bytes, 'type={}'.format(msg_type))

            # This is as consistent as the message defs..
            array_type = msg._slot_types[1]

            expected_tag = TAGGED_ARRAY_FORMATS[array_type][0]
            self.assertEqual(tag.tag, expected_tag, 'type={}'.format(msg_type))

            fmt = TAGGED_ARRAY_FORMATS[array_type][1]
            fmt_to_length = fmt.format(len(msg.data))
            unpacked = list(struct.unpack(fmt_to_length, tag.value))

            self.assertEqual(unpacked, msg.data, 'type={}'.format(msg_type))

    def test_nested_messages(self):
        msg = UInt8MultiArray(layout=MultiArrayLayout(
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
            ]
        ))
        extracted = extract_cbor_values(msg)

        ex_layout = extracted['layout']
        self.assertEqual(type(ex_layout), dict)
        self.assertEqual(ex_layout['data_offset'], msg.layout.data_offset)
        self.assertEqual(len(ex_layout['dim']), len(msg.layout.dim))
        for i, val in enumerate(msg.layout.dim):
            self.assertEqual(ex_layout['dim'][i]['label'], val.label)
            self.assertEqual(ex_layout['dim'][i]['size'], val.size)
            self.assertEqual(ex_layout['dim'][i]['stride'], val.stride)

    def test_unicode_keys(self):
        msg = String(data="foo")
        extracted = extract_cbor_values(msg)

        keys = extracted.keys()
        for key in keys:
            if PYTHON2:
                self.assertEqual(type(key), unicode)  # noqa: F821
            else:
                self.assertEqual(type(key), str)


PKG = 'rosbridge_library'
NAME = 'test_cbor_conversion'
if __name__ == '__main__':
    rostest.unitrun(PKG, NAME, TestCBORConversion)
