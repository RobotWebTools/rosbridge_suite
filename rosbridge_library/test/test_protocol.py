from __future__ import annotations

import time
import unittest
from typing import Any
from unittest import mock

from rosbridge_library.protocol import Protocol


class TestProtocolIncoming(unittest.TestCase):
    """Tests for the buffering of partial/multiple/broken JSON in Protocol.incoming()."""

    def setUp(self) -> None:
        self.proto = Protocol("test_client", mock.Mock())
        self.received: list[dict[str, Any]] = []
        self.proto.operations["test"] = self.received.append
        self.proto.operations["outer"] = self.received.append
        self.proto.operations["inner"] = self.received.append

    def test_single_message(self) -> None:
        self.proto.incoming('{"op": "test", "n": 1}')

        self.assertEqual(self.received, [{"op": "test", "n": 1}])
        self.assertEqual(self.proto.buffer, "")

    def test_message_split_across_calls(self) -> None:
        self.proto.incoming('{"op": "test", "n"')
        self.assertEqual(self.received, [])

        self.proto.incoming(": 1}")

        self.assertEqual(self.received, [{"op": "test", "n": 1}])
        self.assertEqual(self.proto.buffer, "")

    def test_multiple_messages_in_one_call(self) -> None:
        self.proto.incoming('{"op": "test", "n": 1}{"op": "test", "n": 2}')

        self.assertEqual(self.received, [{"op": "test", "n": 1}, {"op": "test", "n": 2}])
        self.assertEqual(self.proto.buffer, "")

    def test_complete_message_followed_by_partial_one(self) -> None:
        self.proto.incoming('{"op": "test", "n": 1}{"op": "test", "n"')
        self.assertEqual(self.received, [{"op": "test", "n": 1}])

        self.proto.incoming(": 2}")

        self.assertEqual(self.received, [{"op": "test", "n": 1}, {"op": "test", "n": 2}])
        self.assertEqual(self.proto.buffer, "")

    def test_leading_garbage_is_dropped(self) -> None:
        self.proto.incoming('garbage{"op": "test", "n": 1}')

        self.assertEqual(self.received, [{"op": "test", "n": 1}])
        self.assertEqual(self.proto.buffer, "")

    def test_nested_object_inside_message(self) -> None:
        self.proto.incoming('{"op": "test", "data": {"a": [1, 2, {"b": "}"}]}}')

        self.assertEqual(self.received, [{"op": "test", "data": {"a": [1, 2, {"b": "}"}]}}])
        self.assertEqual(self.proto.buffer, "")

    def test_objects_without_op_are_skipped(self) -> None:
        self.proto.incoming('{"n": 1}{"op": "test", "n": 2}')

        self.assertEqual(self.received, [{"op": "test", "n": 2}])

    def test_nested_object_with_op_in_incomplete_message(self) -> None:
        # Known limitation: while the outer object is still incomplete, a complete inner
        # object with an "op" field is taken for a message of its own.
        self.proto.incoming('{"op": "outer", "data": {"op": "inner"}')

        self.assertEqual(self.received, [{"op": "inner"}])

    def test_malformed_input_with_many_brackets_is_fast(self) -> None:
        # The fallback used to try every "{" / "}" pair, which is cubic in the buffer size
        # (minutes of CPU for a few kB). No candidate here is a valid message.
        n = 2000

        start = time.perf_counter()
        self.proto.incoming('{"x":' * n + "}" * n)
        elapsed = time.perf_counter() - start

        self.assertEqual(self.received, [])
        self.assertLess(elapsed, 2.0)

    def test_malformed_flat_input_is_fast(self) -> None:
        start = time.perf_counter()
        self.proto.incoming('{"a": 1, ' * 2000)
        elapsed = time.perf_counter() - start

        self.assertEqual(self.received, [])
        self.assertLess(elapsed, 2.0)


if __name__ == "__main__":
    unittest.main()
