from __future__ import annotations

import asyncio
import unittest

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from rosapi import params

# No node by this name is running, so its parameter services are never available.
MISSING_NODE = "/no_such_node"
MISSING_SERVICE = f"{MISSING_NODE}/get_parameters"


class TestParamClientCache(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.node = Node("test_params")
        params.init(self.node)

    def tearDown(self) -> None:
        params._cached_clients.clear()
        self.node.destroy_node()
        rclpy.shutdown()

    def test_cleanup_of_a_freshly_created_client(self) -> None:
        # The cleanup timer compares against the node clock, so a client cached
        # with a default Time() (ClockType.SYSTEM_TIME) used to make every
        # cleanup run raise "Can't subtract times with different clock types".
        params._get_client(MISSING_SERVICE, GetParameters)

        params._cleanup_timer_callback()

    def test_client_of_an_unavailable_service_is_dropped(self) -> None:
        # The client is destroyed when the service turns out to be unavailable,
        # so it must not stay in the cache - it would be handed out again and
        # raise InvalidHandle on the next call.
        with self.assertRaisesRegex(Exception, "is not available"):
            asyncio.run(params._get_param(MISSING_NODE, "some_param"))

        self.assertNotIn(MISSING_SERVICE, params._cached_clients)
