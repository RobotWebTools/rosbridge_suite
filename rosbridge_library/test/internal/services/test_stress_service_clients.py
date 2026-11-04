#!/usr/bin/env python3
"""
Stress tests for concurrent service-call client lifecycle.

Each ``services.call_service`` invocation creates and destroys a ``rclpy.Client``
on a node that the executor is also spinning. If those lifecycle operations are
not serialized against the executor, the wait-set rebuild can race with the
client teardown and leave a handle in an inconsistent state. The symptom is a
subsequent rclpy binding call raising
``TypeError: Object of type 'NoneType' is not an instance of 'capsule'`` on the
same node, which poisons further service calls for the lifetime of the process.

These tests run many concurrent ``ServiceCaller`` threads against a real
service and then probe the executor to make sure it is still healthy.
"""

from __future__ import annotations

import threading
import time
import unittest
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosbridge_library.internal import services


class TestStressServiceClients(unittest.TestCase):
    """Hammer the service-call lifecycle from many worker threads."""

    def setUp(self) -> None:
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.node = Node("test_stress_service_clients")
        self.node.declare_parameter("test_parameter", 1.0)
        self.executor.add_node(self.node)

        self.executor_errors: list[BaseException] = []
        self._stop_spinning = threading.Event()
        self.exec_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self.exec_thread.start()

        # Service the calls below will target.
        self.service_name = self.node.get_name() + "/list_parameters"

    def _spin_executor(self) -> None:
        while not self._stop_spinning.is_set():
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception as exc:  # noqa: PERF203 - keep spinning
                self.executor_errors.append(exc)

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

    def _call_collecting_errors(
        self,
        errors: list[BaseException],
        successes: list[dict[str, Any]],
    ) -> None:
        def success(json_: dict[str, Any]) -> None:
            successes.append(json_)

        def error(exc: Exception) -> None:
            errors.append(exc)

        caller = services.ServiceCaller(
            self.service_name,
            None,
            5.0,
            success,
            error,
            self.node,
        )
        caller.start()
        caller.join(timeout=10)

    def test_many_parallel_service_calls(self) -> None:
        """40 ServiceCallers run concurrently against the same service."""
        num_callers = 40
        errors: list[BaseException] = []
        successes: list[dict[str, Any]] = []

        threads = [
            threading.Thread(
                target=self._call_collecting_errors, args=(errors, successes), daemon=True
            )
            for _ in range(num_callers)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=15)

        self.assertEqual(errors, [], f"Service callers raised: {errors[:5]}")
        self.assertEqual(
            len(successes),
            num_callers,
            f"Expected {num_callers} successes, got {len(successes)}",
        )
        self._assert_executor_healthy()
        self._assert_executor_functional("Executor died after parallel service calls")

    def test_barrier_synchronized_service_calls(self) -> None:
        """20 ServiceCallers start at the same instant via a barrier."""
        num_callers = 20
        barrier = threading.Barrier(num_callers)
        errors: list[BaseException] = []
        successes: list[dict[str, Any]] = []

        def worker() -> None:
            barrier.wait(timeout=10)
            self._call_collecting_errors(errors, successes)

        threads = [threading.Thread(target=worker, daemon=True) for _ in range(num_callers)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=15)

        self.assertEqual(errors, [], f"Service callers raised: {errors[:5]}")
        self.assertEqual(len(successes), num_callers)
        self._assert_executor_healthy()
        self._assert_executor_functional("Executor died after barrier-synced calls")

    def test_repeated_rapid_service_calls(self) -> None:
        """100 sequential calls with no settle time exercise rapid create/destroy."""
        for iteration in range(100):
            errors: list[BaseException] = []
            successes: list[dict[str, Any]] = []
            self._call_collecting_errors(errors, successes)
            self.assertEqual(errors, [], f"Iteration {iteration} raised: {errors}")
            self.assertEqual(len(successes), 1, f"Iteration {iteration} produced no response")
            self._assert_executor_functional(f"Executor died at iteration {iteration}")
        self._assert_executor_healthy()

    def test_interleaved_waves_of_service_calls(self) -> None:
        """Each wave starts before the previous one has finished tearing down clients."""
        wave_size = 10
        waves = 10
        errors: list[BaseException] = []
        successes: list[dict[str, Any]] = []

        prev_threads: list[threading.Thread] = []
        for _ in range(waves):
            new_threads = [
                threading.Thread(
                    target=self._call_collecting_errors,
                    args=(errors, successes),
                    daemon=True,
                )
                for _ in range(wave_size)
            ]
            for t in new_threads:
                t.start()
            # Wait briefly so the previous wave is mid-teardown when this one starts.
            time.sleep(0.05)
            for t in prev_threads:
                t.join(timeout=10)
            prev_threads = new_threads

        for t in prev_threads:
            t.join(timeout=10)

        self.assertEqual(errors, [], f"Service callers raised: {errors[:5]}")
        self.assertEqual(len(successes), wave_size * waves)
        self._assert_executor_healthy()
        self._assert_executor_functional("Executor died after interleaved waves")


if __name__ == "__main__":
    unittest.main()
