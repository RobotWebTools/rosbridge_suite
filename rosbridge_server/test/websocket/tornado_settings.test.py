from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import sleep, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.node import Node

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestTornadoSettings(unittest.TestCase):
    @websocket_test
    async def test_tornado_settings_fails(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        failed_code = 0

        def on_close_handler(wasClean: bool, code: int, reason: str) -> None:
            print(f" hello jumbo client closed: wasClean={wasClean}, code={code}, reason={reason}")
            nonlocal failed_code
            failed_code = code

        ws_client = await make_client()
        ws_client.on_close_handler = on_close_handler

        ws_client.sendJson(
            {
                "op": "call_service",
                "type": "rosbridge_test_msgs/TestArrayRequest",
                "service": "/test_service",
                "args": {
                    "int_values": [0] * 330000
                },  # max default size is 1000000, but because it's sent in uint32 the literal size is lower.
            }
        )

        await sleep(node, 1.0)
        self.assertEqual(failed_code, 0)

        ws_client.sendJson(
            {
                "op": "call_service",
                "type": "rosbridge_test_msgs/TestArrayRequest",
                "service": "/test_service",
                "args": {"int_values": [0] * 335000},
            }
        )

        await sleep(node, 1.0)
        self.assertEqual(failed_code, 1009)
