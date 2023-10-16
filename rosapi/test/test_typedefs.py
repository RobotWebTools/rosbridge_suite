#!/usr/bin/env python
import unittest

import rosapi.objectutils as objectutils

# Globally defined ros_loader, used inside the setUp and teardown functions
ros_loader = None


class TestUtils(unittest.TestCase):
    def setUp(self):
        global ros_loader
        self.original_ros_loader = ros_loader
        ros_loader = self._mock_get_message_instance("default")

    def tearDown(self):
        global ros_loader
        ros_loader = self.original_ros_loader

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

    def test_handle_sequences(self):
        # Test for boolean sequence type
        actual_typedef = objectutils.get_typedef("sequence<boolean>")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)


if __name__ == "__main__":
    unittest.main()
