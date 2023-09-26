#!/usr/bin/env python
import unittest

from rosapi.stringify_field_types import stringify_field_types
from rosbridge_library.internal.ros_loader import InvalidModuleException


class TestObjectUtils(unittest.TestCase):
    def test_stringify_field_types(self):
        self.maxDiff = None

        self.assertRegex(
            stringify_field_types("std_msgs/String"),
            r"(?ms)^string data",
        )

        self.assertRegex(
            stringify_field_types("std_msgs/msg/String"),
            r"(?ms)^string data",
        )

        self.assertRegex(
            stringify_field_types("std_msgs/ByteMultiArray"),
            r"""(?s)
MultiArrayLayout  layout.*
byte\[\]            data.*

================================================================================
MSG: std_msgs/MultiArrayLayout
.*
MultiArrayDimension\[\] dim.*
uint32 data_offset.*

================================================================================
MSG: std_msgs/MultiArrayDimension
.*
string label.*
uint32 size.*
uint32 stride.*
""",
        )
        self.assertRegex(
            stringify_field_types("sensor_msgs/Image"),
            r"""(?s)
std_msgs/Header header.*
.*
uint32 height.*
uint32 width.*
.*
string encoding.*
.*
uint8 is_bigendian.*
uint32 step.*
uint8\[\] data.*

================================================================================
MSG: std_msgs/Header
.*
builtin_interfaces/Time stamp.*
string frame_id

================================================================================
MSG: builtin_interfaces/Time
.*
int32 sec.*
uint32 nanosec
""",
        )
        self.assertRegex(
            stringify_field_types("sensor_msgs/CameraInfo"),
            r"""(?s)
std_msgs/Header header.*
uint32 height.*
uint32 width.*
string distortion_model.*
float64\[\] d.*
float64\[9\]  k.*
float64\[9\]  r.*
float64\[12\] p.*
uint32 binning_x.*
uint32 binning_y.*
RegionOfInterest roi.*

================================================================================
MSG: sensor_msgs/RegionOfInterest
.*
uint32 x_offset.*
uint32 y_offset.*
uint32 height.*
uint32 width.*
bool do_rectify

================================================================================
MSG: std_msgs/Header
.*
builtin_interfaces/Time stamp.*
string frame_id

================================================================================
MSG: builtin_interfaces/Time
.*
int32 sec.*
uint32 nanosec
""",
        )

        self.assertRegex(
            stringify_field_types("shape_msgs/SolidPrimitive"),
            r"""(?s)
uint8 BOX=1.*
uint8 SPHERE=2.*
uint8 CYLINDER=3.*
uint8 CONE=4.*
uint8 type.*
float64\[<=3\] dimensions.*
uint8 BOX_X=0.*
uint8 BOX_Y=1.*
uint8 BOX_Z=2.*
uint8 SPHERE_RADIUS=0.*
uint8 CYLINDER_HEIGHT=0.*
uint8 CYLINDER_RADIUS=1.*
uint8 CONE_HEIGHT=0.*
uint8 CONE_RADIUS=1.*
""",
        )

        self.assertEqual(
            stringify_field_types("geometry_msgs/Quaternion"),
            """\
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
""",
        )

        try:
            # We match against a regex here as the Gid.msg differs between distros:
            # Distros up to humble use 24 bytes, more recent distros use 16 bytes.
            # See https://github.com/ros2/rmw_dds_common/pull/68
            self.assertRegex(
                stringify_field_types("rmw_dds_common/NodeEntitiesInfo"),
                r"""string<=256 node_namespace
string<=256 node_name
Gid\[\] reader_gid_seq
Gid\[\] writer_gid_seq

================================================================================
MSG: rmw_dds_common/Gid
char\[(24|16)\] data
""",
            )
        except InvalidModuleException:
            # This message is not present on older ROS distributions
            pass
