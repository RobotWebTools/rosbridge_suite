# Software License Agreement (BSD License)
#
# Copyright (c) 2026, PickNik Robotics, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of PickNik Robotics, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Helpers for serializing rclpy entity lifecycle with the node's executor.

Creating or destroying entities (clients, action servers, subscriptions,
publishers, ...) on a node from a worker thread is not safe against a
``rclpy`` executor that is concurrently spinning the same node: the executor's
wait-set rebuild can race with the worker thread's entity registration or
teardown and leave the executor holding a handle whose underlying PyCapsule
has been freed. The symptom is either a SIGSEGV inside ``rclpy`` or, less
loudly, a later ``TypeError: Object of type 'NoneType' is not an instance of
'capsule'`` from the next binding call on the affected node.

The remedy used throughout this package — and consistent with the
``IncomingQueue`` fix in #1183 — is to route the lifecycle call through
``executor.create_task(...)`` so it is serialized with the executor's own
work. This module centralizes that pattern.
"""

from __future__ import annotations

from threading import Event
from typing import TYPE_CHECKING, Any, TypeVar

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.node import Node


_T = TypeVar("_T")


def run_on_executor(node_handle: Node, fn: Callable[[], _T]) -> _T:
    """
    Run ``fn()`` on the node's executor thread synchronously and return its result.

    Use this whenever a worker thread needs to create or destroy an rclpy
    entity (``Client``, ``ActionServer``, ``Subscription``, ``Publisher``,
    ...) on a node that an executor is concurrently spinning.

    If the node has no attached executor (rare; mostly unit tests), the
    callable is invoked inline. Exceptions raised by ``fn`` are re-raised in
    the calling thread.
    """
    executor = node_handle.executor
    if executor is None:
        return fn()

    done = Event()
    box: dict[str, Any] = {}

    def _wrapper() -> None:
        try:
            box["result"] = fn()
        except BaseException as exc:
            box["exc"] = exc
        finally:
            done.set()

    executor.create_task(_wrapper)
    done.wait()
    if "exc" in box:
        raise box["exc"]
    return box["result"]


def schedule_on_executor(node_handle: Node, fn: Callable[[], object]) -> None:
    """
    Schedule ``fn()`` for execution on the node's executor thread.

    Fire-and-forget: the caller does not wait for completion. Falls back to
    invoking ``fn`` inline if the node has no attached executor.
    """
    executor = node_handle.executor
    if executor is not None:
        executor.create_task(fn)
    else:
        fn()
