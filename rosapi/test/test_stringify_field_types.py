#!/usr/bin/env python
import unittest

from rosapi.stringify_field_types import stringify_field_types
from rosbridge_library.internal.ros_loader import InvalidModuleException


class TestObjectUtils(unittest.TestCase):
    def test_stringify_field_types(self):
        self.maxDiff = None

        self.assertEqual(stringify_field_types("std_msgs/String"), "string data\n")

        self.assertEqual(stringify_field_types("std_msgs/msg/String"), "string data\n")

        self.assertEqual(
            stringify_field_types("std_msgs/ByteMultiArray"),
            """\
std_msgs/msg/MultiArrayLayout layout
byte[] data

================================================================================
MSG: std_msgs/msg/MultiArrayLayout
std_msgs/msg/MultiArrayDimension[] dim
uint32 data_offset

================================================================================
MSG: std_msgs/msg/MultiArrayDimension
string label
uint32 size
uint32 stride
""",
        )
        self.assertEqual(
            stringify_field_types("sensor_msgs/Image"),
            """\
std_msgs/msg/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

================================================================================
MSG: std_msgs/msg/Header
builtin_interfaces/msg/Time stamp
string frame_id
""",
        )
        self.assertEqual(
            stringify_field_types("sensor_msgs/CameraInfo"),
            """\
std_msgs/msg/Header header
uint32 height
uint32 width
string distortion_model
float64[] d
float64[9] k
float64[9] r
float64[12] p
uint32 binning_x
uint32 binning_y
sensor_msgs/msg/RegionOfInterest roi

================================================================================
MSG: sensor_msgs/msg/RegionOfInterest
uint32 x_offset
uint32 y_offset
uint32 height
uint32 width
bool do_rectify

================================================================================
MSG: std_msgs/msg/Header
builtin_interfaces/msg/Time stamp
string frame_id
""",
        )

        self.assertEqual(
            stringify_field_types("shape_msgs/SolidPrimitive"),
            """\
uint8 type
float64[<=3] dimensions
""",
        )

        try:
            self.assertEqual(
                stringify_field_types("rmw_dds_common/NodeEntitiesInfo"),
                """\
string<=256 node_namespace
string<=256 node_name
rmw_dds_common/msg/Gid[] reader_gid_seq
rmw_dds_common/msg/Gid[] writer_gid_seq

================================================================================
MSG: rmw_dds_common/msg/Gid
uint8[24] data
""",
            )
        except InvalidModuleException:
            # This message is not present on older ROS distributions
            pass
