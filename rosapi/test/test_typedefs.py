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


class TestBoundedTypes(unittest.TestCase):
    """
    Regression: bounded strings and bounded sequences must be normalised.

    IDL spells these `string<255>` and `sequence<T, N>`. Neither was stripped,
    so `string<255>` reached `_type_name` as an unknown type whose value is a
    plain `str`, tripping its `assert isinstance(instance, ROSMessage)` - and
    since rosapi_node runs a bare `rclpy.spin()`, that AssertionError killed the
    node and every `/rosapi/*` service with it. Reproducible against any node:
    `type_description_interfaces/msg/TypeDescription`, reachable from every
    node's `~/get_type_description` service, has `string<255>` fields.
    """

    @staticmethod
    def _mock(field_types: dict[str, str], values: dict[str, object]) -> object:
        class MockMsg:
            __slots__ = ["_" + name for name in field_types]
            _fields_and_field_types: ClassVar = dict(field_types)

            def __init__(self) -> None:
                for name, value in values.items():
                    setattr(self, "_" + name, value)

        return MockMsg()

    def test_bounded_string_is_reported_as_string(self) -> None:
        inst = self._mock({"type_name": "string<255>"}, {"type_name": "some/Type"})

        _, types, lens, _ = objectutils._handle_array_information(inst)

        self.assertEqual(types, ["string"])
        self.assertEqual(lens, [-1])

    def test_bounded_wstring_is_reported_as_wstring(self) -> None:
        inst = self._mock({"label": "wstring<64>"}, {"label": "x"})

        _, types, _, _ = objectutils._handle_array_information(inst)

        self.assertEqual(types, ["wstring"])

    def test_bounded_sequence_drops_the_upper_bound(self) -> None:
        # rcl_interfaces/ParameterDescriptor spells its ranges this way; the
        # bound used to be captured into the type, so the nested typedef lookup
        # searched for a message class literally named "FloatingPointRange, 1".
        inst = self._mock(
            {"floating_point_range": "sequence<rcl_interfaces/FloatingPointRange, 1>"},
            {"floating_point_range": []},
        )

        _, types, lens, _ = objectutils._handle_array_information(inst)

        self.assertEqual(types, ["rcl_interfaces/FloatingPointRange"])
        self.assertEqual(lens, [0])

    def test_sequence_of_bounded_strings(self) -> None:
        inst = self._mock({"names": "sequence<string<255>>"}, {"names": []})

        _, types, lens, _ = objectutils._handle_array_information(inst)

        self.assertEqual(types, ["string"])
        self.assertEqual(lens, [0])

    def test_unbounded_sequence_is_unchanged(self) -> None:
        inst = self._mock({"sequence": "sequence<int32>"}, {"sequence": []})

        _, types, lens, _ = objectutils._handle_array_information(inst)

        self.assertEqual(types, ["int32"])
        self.assertEqual(lens, [0])

    def test_fixed_size_array_is_unchanged(self) -> None:
        inst = self._mock({"uuid": "uint8[16]"}, {"uuid": b"\x00" * 16})

        _, types, lens, _ = objectutils._handle_array_information(inst)

        self.assertEqual(types, ["uint8"])
        self.assertEqual(lens, [16])

    def test_non_message_value_does_not_raise(self) -> None:
        # rclpy hands primitive sequences over as array.array, not list, so
        # _type_name must not assume anything it is given is a message.
        import array

        self.assertEqual(objectutils._type_name("some/Type", array.array("i", [1, 2])), "some/Type")
        self.assertEqual(objectutils._type_name("some/Type", "a string"), "some/Type")
        self.assertEqual(objectutils._type_name("some/Type", b"bytes"), "some/Type")


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
