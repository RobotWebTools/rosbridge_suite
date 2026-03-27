from __future__ import annotations

import sys
import unittest
from pathlib import Path
from typing import TYPE_CHECKING

import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_testing.actions import ReadyToTest
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

from common import sleep, websocket_test

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from common import TestClientProtocol
    from rclpy.node import Node

log.startLogging(sys.stderr)

# Use a short unregister_timeout so we don't have to wait 10 s per unadvertise.
_UNREGISTER_TIMEOUT = 0.5
# How long to wait for ROS discovery to propagate after publisher creation or destruction.
_DISCOVERY_DELAY = 1.0


def generate_test_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_events_executor",
                default_value="false",
                description="Use EventsExecutor instead of SingleThreadedExecutor",
            ),
            launch_ros.actions.Node(
                executable="rosbridge_websocket",
                package="rosbridge_server",
                parameters=[
                    {
                        "port": 0,
                        "use_events_executor": LaunchConfiguration("use_events_executor"),
                        "unregister_timeout": _UNREGISTER_TIMEOUT,
                    }
                ],
            ),
            ReadyToTest(),
        ]
    )


class TestAdvertisePublishPublishers(unittest.TestCase):
    @websocket_test
    async def test_advertise_publish_publishers(
        self, node: Node, make_client: Callable[[], Awaitable[TestClientProtocol]]
    ) -> None:
        # ------------------------------------------------------------------
        # Test 1 - advertise spawns a publisher on the topic.
        # ------------------------------------------------------------------
        ws1 = await make_client()
        ws1.sendJson({"op": "advertise", "topic": "/test_adv_1", "type": "std_msgs/String"})
        await sleep(node, _DISCOVERY_DELAY)

        self.assertGreater(
            node.count_publishers("/test_adv_1"),
            0,
            "advertise should create a publisher on the topic",
        )

        # ------------------------------------------------------------------
        # Test 2 - unadvertise (without ID) removes the publisher when
        # there are no other clients advertising the same topic.
        # ------------------------------------------------------------------
        ws2 = await make_client()
        ws2.sendJson({"op": "advertise", "topic": "/test_adv_2", "type": "std_msgs/String"})
        await sleep(node, _DISCOVERY_DELAY)
        self.assertGreater(node.count_publishers("/test_adv_2"), 0)

        ws2.sendJson({"op": "unadvertise", "topic": "/test_adv_2"})  # no id
        await sleep(node, _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)

        self.assertEqual(
            node.count_publishers("/test_adv_2"),
            0,
            "unadvertise without ID should destroy the publisher when no other clients remain",
        )

        # ------------------------------------------------------------------
        # Test 3 - multiple advertise with different IDs: the publisher must
        # survive until *all* IDs are unadvertised (or one unadvertise with
        # no ID is sent).
        # ------------------------------------------------------------------

        # Part A: advertise twice with id_a and id_b; unadvertising only
        # id_a must leave the publisher alive; removing id_b destroys it.
        ws3 = await make_client()
        ws3.sendJson(
            {"op": "advertise", "topic": "/test_adv_3", "type": "std_msgs/String", "id": "id_a"}
        )
        ws3.sendJson(
            {"op": "advertise", "topic": "/test_adv_3", "type": "std_msgs/String", "id": "id_b"}
        )
        await sleep(node, _DISCOVERY_DELAY)
        self.assertGreater(node.count_publishers("/test_adv_3"), 0)

        # Remove id_a - id_b still active, publisher must survive.
        ws3.sendJson({"op": "unadvertise", "topic": "/test_adv_3", "id": "id_a"})
        await sleep(node, _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)
        self.assertGreater(
            node.count_publishers("/test_adv_3"),
            0,
            "publisher should survive when only one of two advertisement IDs is removed",
        )

        # Remove id_b - no active IDs left, publisher must be destroyed.
        ws3.sendJson({"op": "unadvertise", "topic": "/test_adv_3", "id": "id_b"})
        await sleep(node, _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)
        self.assertEqual(
            node.count_publishers("/test_adv_3"),
            0,
            "publisher should be destroyed after all advertisement IDs are unadvertised",
        )

        # Part B: unadvertise *without* ID clears all IDs at once.
        ws3b = await make_client()
        ws3b.sendJson(
            {"op": "advertise", "topic": "/test_adv_3b", "type": "std_msgs/String", "id": "id_c"}
        )
        ws3b.sendJson(
            {"op": "advertise", "topic": "/test_adv_3b", "type": "std_msgs/String", "id": "id_d"}
        )
        await sleep(node, _DISCOVERY_DELAY)
        self.assertGreater(node.count_publishers("/test_adv_3b"), 0)

        ws3b.sendJson({"op": "unadvertise", "topic": "/test_adv_3b"})  # no id - clears all
        await sleep(node, _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)
        self.assertEqual(
            node.count_publishers("/test_adv_3b"),
            0,
            "unadvertise without ID should destroy the publisher even when multiple IDs were active",
        )

        # ------------------------------------------------------------------
        # Test 4 - publish registers a publisher even when no advertise was
        # called beforehand.
        # ------------------------------------------------------------------
        ws4 = await make_client()
        ws4.sendJson(
            {
                "op": "publish",
                "topic": "/test_pub_4",
                "type": "std_msgs/String",
                "msg": {"data": "hello"},
            }
        )
        await sleep(node, _DISCOVERY_DELAY)

        self.assertGreater(
            node.count_publishers("/test_pub_4"),
            0,
            "publish should register a publisher even without a prior advertise",
        )

        # ------------------------------------------------------------------
        # Test 5 - when a client disconnects, all its publishers are
        # destroyed — both those spawned by advertise and by publish.
        # ------------------------------------------------------------------

        # Case A: publisher spawned by advertise.
        ws5a = await make_client()
        ws5a.sendJson({"op": "advertise", "topic": "/test_disc_adv", "type": "std_msgs/String"})
        await sleep(node, _DISCOVERY_DELAY)
        self.assertGreater(node.count_publishers("/test_disc_adv"), 0)

        ws5a.sendClose()
        # Allow time for the close handshake, finish() to run, and the
        # unregister timer to fire.
        await sleep(node, 2 * _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)
        self.assertEqual(
            node.count_publishers("/test_disc_adv"),
            0,
            "publisher from advertise should be destroyed when the client disconnects",
        )

        # Case B: publisher spawned by publish.
        ws5b = await make_client()
        ws5b.sendJson(
            {
                "op": "publish",
                "topic": "/test_disc_pub",
                "type": "std_msgs/String",
                "msg": {"data": "hi"},
            }
        )
        await sleep(node, _DISCOVERY_DELAY)
        self.assertGreater(node.count_publishers("/test_disc_pub"), 0)

        ws5b.sendClose()
        await sleep(node, 2 * _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)
        self.assertEqual(
            node.count_publishers("/test_disc_pub"),
            0,
            "publisher from publish should be destroyed when the client disconnects",
        )

        # ------------------------------------------------------------------
        # Test 6 - publish can register an advertisement with an ID, which
        # can later be destroyed by unadvertise with the same ID.
        # ------------------------------------------------------------------
        ws6 = await make_client()
        ws6.sendJson(
            {
                "op": "publish",
                "id": "pub_reg_id",
                "topic": "/test_pub_id",
                "type": "std_msgs/String",
                "msg": {"data": "hello"},
            }
        )
        await sleep(node, _DISCOVERY_DELAY)
        self.assertGreater(node.count_publishers("/test_pub_id"), 0)

        ws6.sendJson({"op": "unadvertise", "id": "pub_reg_id", "topic": "/test_pub_id"})
        await sleep(node, _DISCOVERY_DELAY + _UNREGISTER_TIMEOUT)
        self.assertEqual(
            node.count_publishers("/test_pub_id"),
            0,
            "unadvertise with the same ID used in publish should destroy the publisher",
        )
