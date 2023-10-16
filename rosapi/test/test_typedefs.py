#!/usr/bin/env python
import unittest

import rosapi.objectutils as objectutils


class TestUtils(unittest.TestCase):
    def setUp(self):
        # Mock ros_loader.get_message_instance to return test instances
        self.ros_loader_patcher = unittest.mock.patch(
            "rosbridge_library.internal.ros_loader.get_message_instance"
        )
        self.get_message_instance = self.ros_loader_patcher.start()
        self.get_message_instance.side_effect = self._mock_get_message_instance

    def tearDown(self):
        self.ros_loader_patcher.stop()

    def _mock_get_message_instance(self, type):
        mock_instance = unittest.mock.Mock()
        mock_instance.__slots__ = ["_" + type]
        mock_instance._fields_and_field_types = {type: type}
        return mock_instance

    def test_get_typedef_for_atomic_types(self):
        # Test for boolean type
        actual_typedef = objectutils.get_typedef("boolean")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)

        # Test for float type
        actual_typedef = objectutils.get_typedef("float")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)

    def test_handle_complex_types(self):
        # Test for Pose
        actual_typedef = objectutils.get_typedef("Pose")
        self.assertEqual(actual_typedef["type"], "unittest/Mock")
        self.assertEqual(actual_typedef["fieldnames"], ["_Pose"])
        self.assertEqual(actual_typedef["fieldarraylen"], [-1])
        self.assertEqual(actual_typedef["examples"], ["{}"])

    def test_handle_sequences(self):
        # Test for boolean sequence type
        actual_typedef = objectutils.get_typedef("sequence<boolean>")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)

        # Test for Pose sequence type
        actual_typedef = objectutils.get_typedef("sequence<Pose>")
        self.assertEqual(actual_typedef["type"], "unittest/Mock")
        self.assertEqual(actual_typedef["fieldnames"], ["_Pose"])
        self.assertEqual(actual_typedef["fieldarraylen"], [-1])
        self.assertEqual(actual_typedef["examples"], ["{}"])


if __name__ == "__main__":
    unittest.main()
