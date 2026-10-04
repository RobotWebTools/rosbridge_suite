#!/usr/bin/env python3
from __future__ import annotations

import unittest
from unittest.mock import Mock

from rosbridge_library.protocol import Protocol


class TestProtocol(unittest.TestCase):
    def test_missing_op_log_omits_malformed_message(self) -> None:
        node = Mock()
        protocol = Protocol("test-client", node)
        message = '{"msg": {"data": 1}'

        protocol.incoming(message)

        node.get_logger.return_value.error.assert_called_once_with(
            "[Client test-client] Received a message without an op. "
            "All messages require 'op' field with value one of: []."
        )

    def test_missing_op_log_omits_large_payload(self) -> None:
        node = Mock()
        protocol = Protocol("test-client", node)
        handler = Mock()
        protocol.register_operation("publish", handler)
        message = '{"id": "test-message", "msg": "' + "x" * 10000 + '"}'

        protocol.incoming(message)

        node.get_logger.return_value.error.assert_called_once_with(
            "[Client test-client] [id: test-message] Received a message without an op. "
            "All messages require 'op' field with value one of: ['publish']."
        )
        handler.assert_not_called()

    def test_missing_op_with_receiver_logs_current_format_error(self) -> None:
        node = Mock()
        protocol = Protocol("test-client", node)
        message = '{"receiver": "/topic", "id": "test-message", "msg": {"data": 1}}'

        protocol.incoming(message)

        node.get_logger.return_value.error.assert_called_once_with(
            "[Client test-client] [id: test-message] Received a message without an op. "
            "All messages require 'op' field with value one of: []."
        )


if __name__ == "__main__":
    unittest.main()
