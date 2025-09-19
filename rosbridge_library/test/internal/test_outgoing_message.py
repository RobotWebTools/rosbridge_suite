#!/usr/bin/env python3
from __future__ import annotations

import unittest

from std_msgs.msg import String

from rosbridge_library.internal.outgoing_message import OutgoingMessage


class TestOutgoingMessage(unittest.TestCase):
    def test_json_values(self) -> None:
        msg = String(data="foo")
        outgoing = OutgoingMessage(msg)

        result = outgoing.get_json_values()
        self.assertEqual(result["data"], msg.data)

        again = outgoing.get_json_values()
        self.assertTrue(result is again)

    def test_cbor_values(self) -> None:
        msg = String(data="foo")
        outgoing = OutgoingMessage(msg)

        result = outgoing.get_cbor_values()
        self.assertEqual(result["data"], msg.data)

        again = outgoing.get_cbor_values()
        self.assertTrue(result is again)


if __name__ == "__main__":
    unittest.main()
