"""
Regression test for event-loop starvation when a high-rate load client is connected.

The scenario:
  1. A "load" client subscribes to N high-frequency topics.
  2. While messages are flowing, a second "probe" client connects and subscribes
     to a low-rate heartbeat topic.
  3. The test asserts that:
     - The probe client can connect at all (handshake not starved).
     - The probe client receives the expected number of heartbeats within the
       measurement window (delivery not starved).

If the event loop is starved the probe client will either time out during the
WebSocket handshake or receive far fewer heartbeats than expected.

NOTE: in a loopback environment with fast hardware this test may pass even with
a regressed implementation, because socket writes complete near-instantly and the
event loop never accumulates a meaningful backlog.  Reliability improves with
larger payloads (LOAD_PAYLOAD_DOUBLES) and a slower or more loaded machine.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

from launch.actions import ExecuteProcess
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py

from common import make_test_description, sleep, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from launch import LaunchDescription
    from rclpy.node import Node

log.startLogging(sys.stderr)

# Load parameters.
N_LOAD_TOPICS = 20

# Probe parameters.
PROBE_RATE_HZ = 10
PROBE_TOPIC = "/starvation_probe"

# How long to let the load saturate the server before the probe client connects.
# Needs to be long enough for the server's write queues to fill up under load.
LOAD_WARMUP_S = 3.0

# How long after the probe client connects to wait before subscribing.
PROBE_SUBSCRIBE_DELAY_S = 5.0

# Measurement window.
PROBE_WINDOW_S = 5.0

# Minimum fraction of expected heartbeats that must arrive.
# Allow generous margin for scheduling jitter; starvation typically delivers 0.
MIN_DELIVERY_FRACTION = 0.90

_PUBLISHER_SCRIPT = str(Path(__file__).parent / "starvation_load_publisher.py")


def generate_test_description() -> LaunchDescription:
    return make_test_description(
        extra_actions=[ExecuteProcess(cmd=[sys.executable, _PUBLISHER_SCRIPT], output="screen")]
    )


class TestEventLoopStarvation(unittest.TestCase):
    @websocket_test
    async def test_probe_receives_heartbeats_under_load(
        self,
        node: Node,
        make_client: Callable[[], Awaitable[TestClientProtocol]],
    ) -> None:
        # ------------------------------------------------------------------ #
        # Load client: subscribe to all load topics.                          #
        # ------------------------------------------------------------------ #
        load_client = await make_client()
        for i in range(N_LOAD_TOPICS):
            load_client.sendJson(
                {
                    "op": "subscribe",
                    "topic": f"/starvation_load_{i}",
                    "type": "std_msgs/msg/Float64MultiArray",
                }
            )

        # Let the load flow for a while so the event loop is under stress
        # when the probe client attempts to connect.
        await sleep(node, LOAD_WARMUP_S)

        # ------------------------------------------------------------------ #
        # Probe client: connect while load is running.                        #
        # ------------------------------------------------------------------ #
        # If the event loop is starved, make_client() will raise here because
        # the TCP accept / WebSocket handshake never gets scheduled.
        try:
            probe_client = await make_client()
        except Exception as exc:
            self.fail(
                f"Event-loop starvation detected: probe client could not connect under load: {exc}"
            )

        # Wait with the connection open but without subscribing yet, so the
        # load is still flowing hard when we finally ask for probe messages.
        # Mirrors the asyncio.sleep(6) in bridge_starvation_client.py.
        await sleep(node, PROBE_SUBSCRIBE_DELAY_S)

        probe_client.sendJson(
            {
                "op": "subscribe",
                "topic": PROBE_TOPIC,
                "type": "std_msgs/msg/String",
            }
        )

        # ------------------------------------------------------------------ #
        # Count heartbeats delivered during the measurement window.           #
        # ------------------------------------------------------------------ #
        received: list[object] = []

        def on_probe_message(msg: object) -> None:
            if isinstance(msg, dict) and msg.get("topic") == PROBE_TOPIC:
                received.append(msg)

        probe_client.message_handler = on_probe_message

        await sleep(node, PROBE_WINDOW_S)

        # ------------------------------------------------------------------ #
        # Assertions.                                                         #
        # ------------------------------------------------------------------ #
        expected = int(PROBE_RATE_HZ * PROBE_WINDOW_S)
        min_required = int(expected * MIN_DELIVERY_FRACTION)

        self.assertGreaterEqual(
            len(received),
            min_required,
            f"Event-loop starvation detected: received only {len(received)}/{expected} "
            f"probe heartbeats (minimum required: {min_required}). "
            "The event loop is likely blocked by high-rate outgoing message scheduling.",
        )
