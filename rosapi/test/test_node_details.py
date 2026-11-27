from __future__ import annotations

import importlib.machinery
import importlib.util
import unittest
from pathlib import Path
from unittest import mock

import rclpy
from rclpy.node import Node, NodeNameNonExistentError

from rosapi import proxy
from rosapi_msgs.msg import TypeDef
from rosapi_msgs.srv import NodeDetails

# rosapi_node is a script without a .py extension, so load it by path.
_loader = importlib.machinery.SourceFileLoader(
    "rosapi_node", str(Path(__file__).parents[1] / "scripts" / "rosapi_node")
)
_spec = importlib.util.spec_from_loader("rosapi_node", _loader)
assert _spec is not None
rosapi_node = importlib.util.module_from_spec(_spec)
_loader.exec_module(rosapi_node)


class TestNodeDetails(unittest.TestCase):
    def setUp(self) -> None:
        rclpy.init()
        self.node = Node("test_node_details")
        proxy.init(self.node)

    def tearDown(self) -> None:
        self.node.destroy_node()
        rclpy.shutdown()

    def get_node_details(self, node_name: str) -> NodeDetails.Response:
        return rosapi_node.Rosapi.get_node_details(
            None, NodeDetails.Request(node=node_name), NodeDetails.Response()
        )

    def assert_empty(self, response: NodeDetails.Response) -> None:
        self.assertEqual(list(response.subscribing), [])
        self.assertEqual(list(response.publishing), [])
        self.assertEqual(list(response.services), [])

    def test_unknown_node(self) -> None:
        self.assert_empty(self.get_node_details("/no_such_node"))

    def test_node_stops_during_lookup(self) -> None:
        with mock.patch.object(
            proxy,
            "get_node_subscriptions",
            side_effect=NodeNameNonExistentError("node is gone"),
        ):
            self.assert_empty(self.get_node_details("/test_node_details"))

    def test_running_node(self) -> None:
        self.node.create_publisher(TypeDef, "/test_topic", 1)

        response = self.get_node_details("/test_node_details")

        self.assertIn("/test_topic", response.publishing)


if __name__ == "__main__":
    unittest.main()
