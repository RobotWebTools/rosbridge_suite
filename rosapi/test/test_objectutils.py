#!/usr/bin/env python
import unittest

from rosapi.objectutils import get_typedef_full_text


class TestObjectUtils(unittest.TestCase):

    def test_get_typedef_full_text(self):

        self.maxDiff=None
        self.assertEqual(get_typedef_full_text("std_msgs/String"), "string data\n")
        self.assertEqual(get_typedef_full_text("std_msgs/msg/String"), "string data\n")
        self.assertEqual(get_typedef_full_text("std_msgs/ByteMultiArray"), """\
std_msgs/MultiArrayLayout layout
byte[] data

================================================================================
MSG: std_msgs/MultiArrayLayout
std_msgs/MultiArrayDimension[] dim
uint32 data_offset

================================================================================
MSG: std_msgs/MultiArrayDimension
string label
uint32 size
uint32 stride
""")
        self.assertEqual(get_typedef_full_text("sensor_msgs/Image"),
        """\
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
""")
        self.assertEqual(get_typedef_full_text("sensor_msgs/CameraInfo"), 
        """\
std_msgs/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/RegionOfInterest roi

================================================================================
MSG: sensor_msgs/RegionOfInterest
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
""")


