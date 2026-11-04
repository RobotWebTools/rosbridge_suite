from __future__ import annotations

import json
import sys
import unittest
import urllib.request
from importlib.metadata import version
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from twisted.python import log

sys.path.append(str(Path(__file__).parent))  # enable importing from common.py in this directory

import common
from common import get_server_port

log.startLogging(sys.stderr)

generate_test_description = common.generate_test_description


class TestVersionHandler(unittest.TestCase):
    def test_version_endpoint(self) -> None:
        context = rclpy.Context()
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        node = Node("test_version_handler", context=context)
        executor.add_node(node)
        try:
            port_future = executor.create_task(get_server_port, node)
            executor.spin_until_future_complete(port_future, timeout_sec=10.0)
            port = port_future.result()
        finally:
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown(context=context)

        with urllib.request.urlopen(f"http://127.0.0.1:{port}/version") as response:
            assert response.status == 200
            assert response.headers.get_content_type() == "application/json"
            data = json.loads(response.read())

        assert "protocol" in data
        assert "server" in data
        assert "major" in data["protocol"]
        assert "minor" in data["protocol"]
        assert "patch" in data["protocol"]
        assert isinstance(data["protocol"]["major"], int)
        assert isinstance(data["protocol"]["minor"], int)
        assert isinstance(data["protocol"]["patch"], int)
        assert data["server"] == version("rosbridge_server")
