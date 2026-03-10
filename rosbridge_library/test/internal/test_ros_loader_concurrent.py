#!/usr/bin/env python3
"""
Tests for concurrent type loading in ros_loader.

These tests verify that concurrent first-time imports of types from the same
package hierarchy are serialized by _import_lock. Without the lock, multiple
threads can be inside _load_class (i.e. importlib.import_module) simultaneously,
which causes Python's per-module import locks to deadlock when two threads
import sibling submodules from the same package (e.g. rosapi_msgs.srv and
rosapi_msgs.msg both importing rosapi_msgs.msg._type_def).

The tests use unittest.mock to patch _load_class with a slow wrapper that
detects concurrent (overlapping) execution deterministically.
"""

from __future__ import annotations

import threading
import time
import unittest
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from typing import TYPE_CHECKING, NoReturn
from unittest.mock import patch

from rosbridge_library.internal import ros_loader

if TYPE_CHECKING:
    from rosbridge_library.internal.type_support import ROSAction, ROSMessage, ROSService


class TestConcurrentTypeLoading(unittest.TestCase):
    """Test that concurrent loads of related types serialize _load_class calls."""

    def _clear_cache_for(self, typestrings: list[str]) -> None:
        """Remove given typestrings from the ros_loader caches so _load_class is invoked."""
        caches: list[
            dict[str, type[ROSMessage]] | dict[str, type[ROSService]] | dict[str, type[ROSAction]]
        ] = [ros_loader._loaded_msgs, ros_loader._loaded_srvs, ros_loader._loaded_actions]
        for cache in caches:
            for ts in typestrings:
                cache.pop(ts, None)
                splits = [x for x in ts.split("/") if x]
                if len(splits) >= 2:
                    norm = splits[0] + "/" + splits[-1]
                    cache.pop(norm, None)

    def test_load_class_calls_are_serialized(self) -> None:
        """
        Verify that _load_class is never executing in two threads at once.

        Wrap the real _load_class with a spy that tracks concurrent entry.
        """
        real_load_class = ros_loader._load_class
        concurrency_counter = 0
        max_concurrency = 0
        lock = threading.Lock()
        overlap_detected = threading.Event()

        def slow_load_class(
            modname: str,
            subname: str,
            classname: str,
        ) -> type[ROSMessage | ROSService | ROSAction]:
            nonlocal concurrency_counter, max_concurrency
            with lock:
                concurrency_counter += 1
                max_concurrency = max(max_concurrency, concurrency_counter)
                if concurrency_counter > 1:
                    overlap_detected.set()
            try:
                # Small delay to widen the window for concurrent entry
                time.sleep(0.05)
                return real_load_class(modname, subname, classname)
            finally:
                with lock:
                    concurrency_counter -= 1

        msg_types = [
            "std_msgs/Bool",
            "std_msgs/String",
            "std_msgs/Int32",
            "std_msgs/Float64",
            "std_msgs/Header",
            "std_msgs/ColorRGBA",
            "std_msgs/UInt8",
            "std_msgs/Empty",
        ]

        self._clear_cache_for(msg_types)

        with (
            patch.object(ros_loader, "_load_class", side_effect=slow_load_class),
            ThreadPoolExecutor(max_workers=len(msg_types)) as pool,
        ):
            futures = {pool.submit(ros_loader.get_message_class, ts): ts for ts in msg_types}
            for future in as_completed(futures, timeout=30):
                future.result(timeout=30)

        self.assertFalse(
            overlap_detected.is_set(),
            f"_load_class was called concurrently (max concurrency: {max_concurrency}). "
            "This means imports are not serialized, which can cause deadlocks "
            "when multiple threads import from the same package hierarchy.",
        )

    def test_mixed_msg_and_srv_loads_are_serialized(self) -> None:
        """Verify serialization when loading both msg and srv types concurrently."""
        real_load_class = ros_loader._load_class
        concurrency_counter = 0
        max_concurrency = 0
        lock = threading.Lock()
        overlap_detected = threading.Event()

        def slow_load_class(
            modname: str,
            subname: str,
            classname: str,
        ) -> type[ROSMessage | ROSService | ROSAction]:
            nonlocal concurrency_counter, max_concurrency
            with lock:
                concurrency_counter += 1
                max_concurrency = max(max_concurrency, concurrency_counter)
                if concurrency_counter > 1:
                    overlap_detected.set()
            try:
                time.sleep(0.05)
                return real_load_class(modname, subname, classname)
            finally:
                with lock:
                    concurrency_counter -= 1

        # rcl_interfaces has both msg and srv types, exercising cross-submodule imports
        msg_types = [
            "rcl_interfaces/ParameterValue",
            "rcl_interfaces/Parameter",
            "rcl_interfaces/ParameterType",
        ]
        srv_types = [
            "rcl_interfaces/GetParameters",
            "rcl_interfaces/SetParameters",
            "rcl_interfaces/ListParameters",
        ]
        all_types = msg_types + srv_types

        self._clear_cache_for(all_types)

        with (
            patch.object(ros_loader, "_load_class", side_effect=slow_load_class),
            ThreadPoolExecutor(max_workers=len(all_types)) as pool,
        ):
            futures: dict[Future[type[ROSMessage | ROSService]], str] = {}
            for ts in msg_types:
                futures[pool.submit(ros_loader.get_message_class, ts)] = ts
            for ts in srv_types:
                futures[pool.submit(ros_loader.get_service_class, ts)] = ts

            for future in as_completed(futures, timeout=30):
                future.result(timeout=30)

        self.assertFalse(
            overlap_detected.is_set(),
            f"_load_class was called concurrently for mixed msg/srv types "
            f"(max concurrency: {max_concurrency}). "
            "This means imports are not serialized, which can cause deadlocks "
            "when multiple threads import from the same package hierarchy.",
        )

    def test_cached_types_bypass_import_lock(self) -> None:
        """Verify that already-cached types return immediately without _load_class."""
        # Pre-warm the cache
        types = ["std_msgs/Bool", "std_msgs/String", "std_msgs/Int32"]
        for ts in types:
            ros_loader.get_message_class(ts)

        load_class_called = threading.Event()

        def fail_load_class(_modname: str, _subname: str, _classname: str) -> NoReturn:
            load_class_called.set()
            msg = "_load_class should not be called for cached types"
            raise AssertionError(msg)

        with (
            patch.object(ros_loader, "_load_class", side_effect=fail_load_class),
            ThreadPoolExecutor(max_workers=len(types)) as pool,
        ):
            futures = {pool.submit(ros_loader.get_message_class, ts): ts for ts in types}
            for future in as_completed(futures, timeout=10):
                cls = future.result(timeout=10)
                self.assertIsNotNone(cls)

        self.assertFalse(
            load_class_called.is_set(),
            "_load_class was called despite types being cached",
        )
