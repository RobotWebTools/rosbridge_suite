#!/usr/bin/env python3
"""Stress tests for concurrent client creation and destruction."""

from __future__ import annotations

import json
import threading
import time
import unittest
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_server.websocket_handler import IncomingQueue


def _subscribe_msg(topic: str) -> str:
    return json.dumps({"op": "subscribe", "topic": topic, "type": "std_msgs/msg/String"})


def _advertise_msg(topic: str) -> str:
    return json.dumps({"op": "advertise", "topic": topic, "type": "std_msgs/msg/String"})


class TestStressClients(unittest.TestCase):
    """Stress test client lifecycle operations against the executor."""

    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_stress_clients")
        self.executor.add_node(self.node)

        self.executor_errors: list[BaseException] = []
        self._stop_spinning = threading.Event()
        self.exec_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self.exec_thread.start()

        self._test_id = uuid.uuid4().hex[:8]

    def _spin_executor(self) -> None:
        while not self._stop_spinning.is_set():
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception as e:  # noqa: PERF203
                self.executor_errors.append(e)

    def tearDown(self) -> None:
        self._stop_spinning.set()
        self.exec_thread.join(timeout=10)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def _assert_executor_healthy(self) -> None:
        self.assertTrue(self.exec_thread.is_alive(), "Executor thread died")
        self.assertEqual(
            self.executor_errors,
            [],
            f"Executor raised {len(self.executor_errors)} error(s): {self.executor_errors[:5]}",
        )

    def _assert_executor_functional(self, msg: str = "Executor cannot process tasks") -> None:
        done = threading.Event()
        self.executor.create_task(done.set)
        self.assertTrue(done.wait(timeout=5), msg)

    def _make_client(self, num_subs: int, num_pubs: int, prefix: str) -> IncomingQueue:
        protocol = RosbridgeProtocol(str(uuid.uuid4()), self.node)
        queue = IncomingQueue(protocol)
        queue.start()
        for i in range(num_subs):
            queue.push(_subscribe_msg(f"/{prefix}_s{i}"))
        for i in range(num_pubs):
            queue.push(_advertise_msg(f"/{prefix}_p{i}"))
        return queue

    def test_barrier_synchronized_mass_disconnect(self) -> None:
        """30 clients disconnect at the exact same instant via a barrier."""
        num_clients = 30
        entities_per_client = 20

        queues = [
            self._make_client(
                entities_per_client, entities_per_client, f"barrier_{self._test_id}_c{i}"
            )
            for i in range(num_clients)
        ]

        time.sleep(3.0)
        self._assert_executor_healthy()

        barrier = threading.Barrier(num_clients)
        barrier_errors: list[Exception] = []

        def _finish_at_barrier(q: IncomingQueue) -> None:
            try:
                barrier.wait(timeout=10)
                q.finish()
            except Exception as e:
                barrier_errors.append(e)

        threads = [threading.Thread(target=_finish_at_barrier, args=(q,)) for q in queues]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=15)
        for q in queues:
            q.join(timeout=10)

        self.assertFalse(barrier_errors, f"Barrier/finish errors: {barrier_errors}")
        time.sleep(2.0)
        self._assert_executor_healthy()

        for probe in range(10):
            self._assert_executor_functional(f"Executor dead (probe {probe})")

    def test_repeated_rapid_connect_disconnect(self) -> None:
        """100 rapid create/destroy cycles with no settle time between them."""
        for iteration in range(100):
            q = self._make_client(10, 10, f"rapid_{self._test_id}_{iteration}")
            q.finish()
            q.join(timeout=5)
            self._assert_executor_functional(f"Executor died at iteration {iteration}")

        self._assert_executor_healthy()

    def test_interleaved_connect_disconnect(self) -> None:
        """New clients connect while old clients disconnect each wave."""
        previous_queues: list[IncomingQueue] = []

        for wave in range(20):
            new_queues = [
                self._make_client(10, 0, f"wave_{self._test_id}_w{wave}_c{i}") for i in range(10)
            ]

            for q in previous_queues:
                q.finish()
            for q in previous_queues:
                q.join(timeout=5)

            previous_queues = new_queues
            self._assert_executor_functional(f"Executor died at wave {wave}")

        for q in previous_queues:
            q.finish()
            q.join(timeout=5)

        time.sleep(1.0)
        self._assert_executor_healthy()

    def test_combined_stress(self) -> None:
        """Three phases: barrier-disconnect batch A while creating B, rapid cycling, then destroy B."""
        # Phase 1: barrier-disconnect batch A while creating batch B
        batch_a = [self._make_client(15, 15, f"combo_{self._test_id}_a{i}") for i in range(15)]
        time.sleep(2.0)

        barrier = threading.Barrier(len(batch_a))
        barrier_errors: list[Exception] = []

        def _finish_at_barrier(q: IncomingQueue) -> None:
            try:
                barrier.wait(timeout=10)
                q.finish()
            except Exception as e:
                barrier_errors.append(e)

        destroy_threads = [threading.Thread(target=_finish_at_barrier, args=(q,)) for q in batch_a]
        for t in destroy_threads:
            t.start()

        batch_b = [self._make_client(15, 15, f"combo_{self._test_id}_b{i}") for i in range(15)]

        for t in destroy_threads:
            t.join(timeout=15)
        for q in batch_a:
            q.join(timeout=10)

        self.assertFalse(barrier_errors, f"Phase 1 errors: {barrier_errors}")
        self._assert_executor_functional("Executor dead after phase 1")

        # Phase 2: rapid cycling while batch B is alive
        for iteration in range(50):
            q = self._make_client(5, 5, f"combo_{self._test_id}_r{iteration}")
            q.finish()
            q.join(timeout=5)

        self._assert_executor_functional("Executor dead after phase 2")

        # Phase 3: destroy batch B
        for q in batch_b:
            q.finish()
        for q in batch_b:
            q.join(timeout=10)

        time.sleep(1.0)
        self._assert_executor_healthy()
        self._assert_executor_functional("Executor dead after phase 3")


if __name__ == "__main__":
    unittest.main()
