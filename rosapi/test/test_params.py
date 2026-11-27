from __future__ import annotations

import asyncio
import unittest
from concurrent.futures import Future
from typing import ClassVar
from unittest import mock

import rclpy
from rcl_interfaces.srv import GetParameters, ListParameters
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

    def _backdate(self, service_name: str) -> None:
        """Age a cached client past the persistence window."""
        params._cached_clients[service_name].last_used_time -= params._client_persistence_sec + 1

    def test_cleanup_of_a_freshly_created_client(self) -> None:
        # A client used to be cached with a default Time() (ClockType.SYSTEM_TIME)
        # while the cleanup timer compared against the node clock, which is a
        # ROSClock (ClockType.ROS_TIME) whether or not use_sim_time is set. That
        # subtraction raises "Can't subtract times with different clock types",
        # out of a timer callback, which takes the whole node down.
        params._get_client(MISSING_SERVICE, GetParameters)

        params._cleanup_timer_callback()

    def test_cleanup_after_an_unavailable_service(self) -> None:
        # The path that actually reached the comparison above: the caller bumps
        # use_count right after _get_client, so the cleanup timer's short circuit
        # normally hides a freshly cached client - except when the service turns
        # out to be unavailable and the call returns in between.
        with self.assertRaisesRegex(Exception, "is not available"):
            asyncio.run(params._get_param(MISSING_NODE, "some_param"))

        params._cleanup_timer_callback()

    def test_client_of_an_unavailable_service_is_dropped(self) -> None:
        # The client is destroyed when the service turns out to be unavailable,
        # so it must not stay in the cache - it would be handed out again and
        # raise InvalidHandle on the next call.
        with self.assertRaisesRegex(Exception, "is not available"):
            asyncio.run(params._get_param(MISSING_NODE, "some_param"))

        self.assertNotIn(MISSING_SERVICE, params._cached_clients)

    def test_idle_client_is_evicted(self) -> None:
        params._get_client(MISSING_SERVICE, GetParameters)
        self._backdate(MISSING_SERVICE)

        params._cleanup_timer_callback()

        self.assertNotIn(MISSING_SERVICE, params._cached_clients)

    def test_client_of_an_ongoing_call_is_kept(self) -> None:
        params._get_client(MISSING_SERVICE, GetParameters)
        self._backdate(MISSING_SERVICE)
        params._cached_clients[MISSING_SERVICE].use_count = 1

        params._cleanup_timer_callback()

        self.assertIn(MISSING_SERVICE, params._cached_clients)


class TestGetParamNames(unittest.TestCase):
    NODE_PARAMS: ClassVar[dict[str, list[str]]] = {
        "/talker": ["use_sim_time", "rate"],
        "/listener": ["use_sim_time", "queue_size"],
    }

    def setUp(self) -> None:
        rclpy.init()
        self.node = Node("test_get_param_names")
        params.init(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    def _fake_client(self, service_name: str) -> mock.Mock:
        node_name = service_name.removesuffix("/list_parameters")
        response = ListParameters.Response()
        response.result.names = self.NODE_PARAMS[node_name]
        future: Future = Future()
        future.set_result(response)

        client = mock.Mock()
        client.service_is_ready.return_value = True
        client.call_async.return_value = future
        return client

    def _get_param_names(self, params_glob: list[str] | None) -> list[str]:
        async def no_wait(*_args: object) -> None:
            pass

        with (
            mock.patch.object(params, "get_nodes", return_value=list(self.NODE_PARAMS)),
            mock.patch.object(params, "futures_wait_for", no_wait),
            mock.patch.object(
                self.node,
                "create_client",
                side_effect=lambda _type, name, **_kw: self._fake_client(name),
            ),
            mock.patch.object(self.node, "destroy_client"),
        ):
            return asyncio.run(params.get_param_names(params_glob))

    def test_no_glob_returns_everything(self) -> None:
        self.assertEqual(
            self._get_param_names(None),
            [
                "/listener:queue_size",
                "/listener:use_sim_time",
                "/talker:rate",
                "/talker:use_sim_time",
            ],
        )

    def test_glob_filters_parameters(self) -> None:
        self.assertEqual(
            self._get_param_names(["/talker:*"]), ["/talker:rate", "/talker:use_sim_time"]
        )

    def test_any_matching_glob_keeps_the_parameter(self) -> None:
        self.assertEqual(
            self._get_param_names(["/talker:rate", "*:queue_size"]),
            ["/listener:queue_size", "/talker:rate"],
        )

    def test_empty_glob_list_matches_nothing(self) -> None:
        # "[]" in the server config means "allow nothing", not "no filtering".
        self.assertEqual(self._get_param_names([]), [])

    def test_glob_matching_nothing_returns_empty(self) -> None:
        self.assertEqual(self._get_param_names(["/other:*"]), [])
