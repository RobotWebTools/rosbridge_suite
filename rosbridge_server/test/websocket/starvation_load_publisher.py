#!/usr/bin/env python3
"""
Standalone publisher node for the event-loop starvation regression test.

Runs as a separate process launched alongside rosbridge so that the load and
probe topics are produced independently of the test process.
"""

from __future__ import annotations

import rclpy
from std_msgs.msg import Float64MultiArray, String

N_LOAD_TOPICS = 20
LOAD_RATE_HZ = 100
LOAD_PAYLOAD_DOUBLES = 2048
PROBE_RATE_HZ = 10
PROBE_TOPIC = "/starvation_probe"


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("starvation_load_publisher")

    load_pubs = [
        node.create_publisher(Float64MultiArray, f"/starvation_load_{i}", 10)
        for i in range(N_LOAD_TOPICS)
    ]
    probe_pub = node.create_publisher(String, PROBE_TOPIC, 10)

    load_msg = Float64MultiArray(data=[0.0] * LOAD_PAYLOAD_DOUBLES)
    probe_msg = String(data="heartbeat")

    def _publish_load() -> None:
        for pub in load_pubs:
            pub.publish(load_msg)

    node.create_timer(1.0 / LOAD_RATE_HZ, _publish_load)
    node.create_timer(1.0 / PROBE_RATE_HZ, lambda: probe_pub.publish(probe_msg))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
