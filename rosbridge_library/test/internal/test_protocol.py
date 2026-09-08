#!/usr/bin/env python3
from __future__ import annotations

import unittest
from unittest.mock import Mock

from rosbridge_library.protocol import Protocol


class TestProtocol(unittest.TestCase):
    def test_missing_op_log_includes_malformed_message(self) -> None:
        node = Mock()
        protocol = Protocol("test-client", node)
        message = '{"msg": {"data": 1}'

        protocol.incoming(message)

        node.get_logger.return_value.error.assert_called_once()
        logged_message = node.get_logger.return_value.error.call_args.args[0]
        self.assertIn(f"Original message was: {message}", logged_message)


if __name__ == "__main__":
    unittest.main()
