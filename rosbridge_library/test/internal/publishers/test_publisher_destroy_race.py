"""Stress test for publisher teardown against a spinning executor."""

from __future__ import annotations

import threading
import time
import unittest
import uuid

import rclpy
from rclpy.exceptions import InvalidHandle
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.internal.publishers import manager


class TestPublisherDestroyRace(unittest.TestCase):
    """PublisherManager destroys publishers on a Timer thread; the executor must survive that."""

    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_publisher_destroy_race")
        # Keeps the executor rebuilding its wait set, which is where a concurrent
        # destroy_publisher used to raise InvalidHandle.
        self.node.create_timer(0.001, lambda: None)
        self.executor.add_node(self.node)

        self.executor_errors: list[BaseException] = []
        self._stop_spinning = threading.Event()
        self.exec_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self.exec_thread.start()

        self._previous_timeout = manager.unregister_timeout
        manager.unregister_timeout = 0.0
        self._test_id = uuid.uuid4().hex[:8]

    def tearDown(self) -> None:
        manager.unregister_timeout = self._previous_timeout
        self._stop_spinning.set()
        self.exec_thread.join(timeout=5.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def _spin_executor(self) -> None:
        while not self._stop_spinning.is_set():
            try:
                self.executor.spin_once(timeout_sec=0.01)
            except InvalidHandle as exc:
                self.executor_errors.append(exc)
                return
            except Exception as exc:  # noqa: BLE001
                self.executor_errors.append(exc)
                return

    def test_rapid_register_unregister_keeps_executor_alive(self) -> None:
        """Register and unregister publishers in a tight loop while the executor spins."""
        client = f"client_{self._test_id}"
        rounds, per_round = 150, 8
        for i in range(rounds):
            topics = [f"/destroy_race_{self._test_id}_{i}_{k}" for k in range(per_round)]
            for topic in topics:
                manager.register(client, topic, self.node, "std_msgs/String")
            time.sleep(0.002)
            for topic in topics:
                manager.unregister(client, topic)
            time.sleep(0.002)
            self.assertFalse(
                self.executor_errors,
                f"executor died after {i + 1} rounds: {self.executor_errors[0]!r}",
            )

        # Let the Timer threads fire and the executor run the scheduled destroys.
        deadline = time.monotonic() + 5.0
        while manager.unregister_timers and time.monotonic() < deadline:
            time.sleep(0.01)
        time.sleep(0.2)
        self.assertFalse(manager.unregister_timers)
        self.assertFalse(
            [topic for topic in manager._publishers if self._test_id in topic],
            "publishers should be unregistered once their timers fired",
        )
        self.assertFalse(self.executor_errors, repr(self.executor_errors))
        self.assertTrue(self.exec_thread.is_alive())


if __name__ == "__main__":
    unittest.main()
