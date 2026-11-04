from __future__ import annotations

import unittest
from typing import ClassVar
from unittest.mock import patch

from rosbridge_library.internal import ros_loader as _ros_loader

from rosapi import objectutils

# Globally defined ros_loader, used inside the setUp and teardown functions
ros_loader = None


class TestUtils(unittest.TestCase):
    def setUp(self) -> None:
        global ros_loader
        self.original_ros_loader = ros_loader
        ros_loader = self._mock_get_message_instance("default")

    def tearDown(self) -> None:
        global ros_loader
        ros_loader = self.original_ros_loader

    def _mock_get_message_instance(self, type_name: str) -> object:
        class MockInstance:
            __slots__ = ["_" + type_name]
            _fields_and_field_types: ClassVar = {type_name: type_name}

        return MockInstance()

    def test_get_typedef_for_atomic_types(self) -> None:
        # Test for boolean type
        actual_typedef = objectutils.get_typedef("boolean")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)

        # Test for float type
        actual_typedef = objectutils.get_typedef("float")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)

    def test_handle_sequences(self) -> None:
        # Test for boolean sequence type
        actual_typedef = objectutils.get_typedef("sequence<boolean>")
        # should be None for an atomic
        self.assertEqual(actual_typedef, None)

    def test_skip_private_slots_in_array_info(self) -> None:
        # create a fake msg with one real field ('data') and one internal slot
        class MockMsg:
            __slots__ = ["_check_fields", "_important_data"]
            _fields_and_field_types: ClassVar = {"important_data": "int32"}

            def __init__(self) -> None:
                self._important_data = 123
                self._check_fields = None

        inst = MockMsg()
        # call the private helper directly
        names, types, lens, examples = objectutils._handle_array_information(inst)

        # should only see our single '_important_data' field
        self.assertEqual(names, ["important_data"])
        # raw type should be 'int32' (no array)
        self.assertEqual(types, ["int32"])
        self.assertEqual(lens, [-1])
        # example should be the stringified value of 123
        self.assertEqual(examples, ["123"])


def _make_invalid_module_exc(pkg: str, subname: str) -> _ros_loader.InvalidModuleException:
    return _ros_loader.InvalidModuleException(
        pkg, subname, ModuleNotFoundError(f"No module named '{pkg}'")
    )


class TestInvalidTypeHandling(unittest.TestCase):
    """Regression: objectutils must not propagate ros_loader exceptions to callers."""

    def test_get_typedef_invalid_type_string_returns_none(self) -> None:
        exc = _ros_loader.InvalidTypeStringException("bad/type")
        with patch.object(_ros_loader, "get_message_instance", side_effect=exc):
            self.assertIsNone(objectutils.get_typedef("bad/type"))

    def test_service_request_typedef_returns_none_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "srv")
        with patch.object(_ros_loader, "get_service_request_instance", side_effect=exc):
            self.assertIsNone(objectutils.get_service_request_typedef("nonexistent_pkg/srv/Fake"))

    def test_service_response_typedef_returns_none_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "srv")
        with patch.object(_ros_loader, "get_service_response_instance", side_effect=exc):
            self.assertIsNone(objectutils.get_service_response_typedef("nonexistent_pkg/srv/Fake"))

    def test_service_request_typedef_recursive_returns_empty_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "srv")
        with patch.object(_ros_loader, "get_service_request_instance", side_effect=exc):
            result = objectutils.get_service_request_typedef_recursive("nonexistent_pkg/srv/Fake")
        self.assertEqual(result, [])

    def test_service_response_typedef_recursive_returns_empty_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "srv")
        with patch.object(_ros_loader, "get_service_response_instance", side_effect=exc):
            result = objectutils.get_service_response_typedef_recursive("nonexistent_pkg/srv/Fake")
        self.assertEqual(result, [])

    def test_action_goal_typedef_recursive_returns_empty_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "action")
        with patch.object(_ros_loader, "get_action_goal_instance", side_effect=exc):
            result = objectutils.get_action_goal_typedef_recursive(
                "nonexistent_pkg/action/FakeAction"
            )
        self.assertEqual(result, [])

    def test_action_result_typedef_recursive_returns_empty_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "action")
        with patch.object(_ros_loader, "get_action_result_instance", side_effect=exc):
            result = objectutils.get_action_result_typedef_recursive(
                "nonexistent_pkg/action/FakeAction"
            )
        self.assertEqual(result, [])

    def test_action_feedback_typedef_recursive_returns_empty_on_bad_package(self) -> None:
        exc = _make_invalid_module_exc("nonexistent_pkg", "action")
        with patch.object(_ros_loader, "get_action_feedback_instance", side_effect=exc):
            result = objectutils.get_action_feedback_typedef_recursive(
                "nonexistent_pkg/action/FakeAction"
            )
        self.assertEqual(result, [])


if __name__ == "__main__":
    unittest.main()
