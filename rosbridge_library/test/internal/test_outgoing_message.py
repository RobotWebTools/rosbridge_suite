#!/usr/bin/env python3
from __future__ import annotations

import unittest

from cbor2 import loads
from rosbridge_library.internal.outgoing_message import OutgoingMessage
from std_msgs.msg import String


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

    def test_cbor_encoding(self) -> None:
        msg = String(data="foo")
        outgoing = OutgoingMessage(msg)

        outgoing_msg = {"op": "publish", "topic": "/chatter"}
        cbor1 = outgoing.get_cbor(outgoing_msg)
        cbor2 = outgoing.get_cbor(outgoing_msg)
        self.assertTrue(cbor1 is cbor2)
        self.assertIsInstance(cbor1, bytes)

        decoded = loads(cbor1)
        self.assertEqual(decoded["op"], "publish")
        self.assertEqual(decoded["topic"], "/chatter")
        self.assertIn("msg", decoded)
        self.assertEqual(decoded["msg"]["data"], msg.data)


if __name__ == "__main__":
    unittest.main()
