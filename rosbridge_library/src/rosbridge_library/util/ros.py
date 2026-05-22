# Software License Agreement (BSD License)
#
# Copyright (c) 2023, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

from __future__ import annotations

from threading import Event
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rclpy.executors import Executor
    from rclpy.node import Node


def is_topic_published(node: Node, topic_name: str) -> bool:
    """
    Check if a node has at least one publisher on the given topic.

    Reads ``node.publishers`` (the local entity list) rather than the rmw graph
    cache. The local list is populated synchronously inside ``create_publisher``
    and cleared inside ``destroy_publisher``, so the result is deterministic
    with respect to the calling thread — no DDS-discovery round-trip is
    involved.
    """
    return any(pub.topic_name == topic_name for pub in node.publishers)


def is_topic_subscribed(node: Node, topic_name: str) -> bool:
    """
    Check if a node has at least one subscription on the given topic.

    Reads ``node.subscriptions`` (the local entity list) rather than the rmw
    graph cache; see ``is_topic_published`` for why this matters.
    """
    return any(sub.topic_name == topic_name for sub in node.subscriptions)


def wait_for_executor_idle(executor: Executor, timeout: float = 5.0) -> None:
    """
    Block until all tasks already queued on ``executor`` have been processed.

    Used by tests that schedule work on the executor and then need to assert
    on the post-condition from a different thread. Submits a no-op task and
    waits for it to run: tasks are FIFO on ``SingleThreadedExecutor``, so when
    the no-op completes every task enqueued before it has also completed.

    Raises ``TimeoutError`` if the executor does not drain within ``timeout``
    seconds, which usually means it is not being spun.
    """
    done = Event()
    executor.create_task(done.set)
    if not done.wait(timeout=timeout):
        msg = f"Executor did not become idle within {timeout}s"
        raise TimeoutError(msg)
