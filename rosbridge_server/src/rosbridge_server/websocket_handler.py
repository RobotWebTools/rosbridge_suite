# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

import asyncio
import sys
import threading
import traceback
import uuid
from asyncio.events import AbstractEventLoop
from collections import deque
from functools import wraps
from typing import TYPE_CHECKING, Any, ClassVar, ParamSpec, TypeVar

from rclpy.node import Node
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from tornado.websocket import WebSocketClosedError, WebSocketHandler

if TYPE_CHECKING:
    from collections.abc import Callable

    from .client_manager import ClientManager


def _log_exception() -> None:
    """Log the most recent exception to ROS."""
    exc = traceback.format_exception(*sys.exc_info())
    node_handle = RosbridgeWebSocket.node_handle
    assert isinstance(node_handle, Node), "Node handle was not set"
    node_handle.get_logger().error("".join(exc))


P = ParamSpec("P")
R = TypeVar("R")


def log_exceptions(f: Callable[P, R]) -> Callable[P, R]:
    """Log exceptions to ROS."""

    @wraps(f)
    def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
        try:
            return f(*args, **kwargs)
        except Exception:
            _log_exception()
            raise

    return wrapper


class IncomingQueue(threading.Thread):
    """Decouples incoming messages from the AsyncIO event loop."""

    def __init__(self, protocol: RosbridgeProtocol, max_queue_size: int | None = None) -> None:
        threading.Thread.__init__(self)
        self.daemon = True
        self.queue: deque[str] = deque(maxlen=max_queue_size)
        self.protocol = protocol

        self.cond = threading.Condition()
        self._finished = False

    def finish(self) -> None:
        """Clear the queue and do not accept further messages."""
        with self.cond:
            self._finished = True
            while len(self.queue) > 0:
                self.queue.popleft()
            self.cond.notify()

    def push(self, msg: str) -> None:
        with self.cond:
            if self.queue.maxlen is not None and len(self.queue) == self.queue.maxlen:
                self.protocol.node_handle.get_logger().warning(
                    "Incoming queue full, dropping oldest message",
                    throttle_duration_sec=1.0,
                )
            self.queue.append(msg)
            self.cond.notify()

    def run(self) -> None:
        while True:
            with self.cond:
                while len(self.queue) == 0 and not self._finished:
                    self.cond.wait()

                if self._finished:
                    break

                msg = self.queue.popleft()

            self.protocol.incoming(msg)
        executor = self.protocol.node_handle.executor
        if executor is not None:
            executor.create_task(self.protocol.finish)
        else:
            self.protocol.finish()


class RosbridgeWebSocket(WebSocketHandler):
    # Class variable to track the number of connected client
    clients_connected: ClassVar[int] = 0

    # Class variable to manage connected clients
    client_manager: ClassVar[ClientManager | None] = None

    # Event loop to run the coroutines on
    event_loop: ClassVar[AbstractEventLoop | None] = None

    # Node handle to pass to RosbridgeProtocol when opening a connection
    node_handle: ClassVar[Node | None] = None

    # Parameters to pass to RosbridgeProtocol when opening a connection
    protocol_parameters: ClassVar[dict[str, Any]] = {}

    # Parameters for the WebSocket handler
    incoming_queue_size: ClassVar[int] = 1000
    write_queue_size: ClassVar[int] = 1000
    use_compression: ClassVar[bool] = False

    # Instance variables for each connection
    client_id: uuid.UUID
    protocol: RosbridgeProtocol
    incoming_queue: IncomingQueue
    write_queue: asyncio.Queue
    write_slots: threading.Semaphore
    write_task: asyncio.Task

    @log_exceptions
    def open(self, *args: str, **kwargs: str) -> None:  # noqa: ARG002
        cls = self.__class__
        assert isinstance(cls.node_handle, Node), "Node handle was not set"
        try:
            self.client_id = uuid.uuid4()
            self.protocol = RosbridgeProtocol(
                self.client_id, cls.node_handle, parameters=cls.protocol_parameters
            )
            self.incoming_queue = IncomingQueue(self.protocol, cls.incoming_queue_size)
            self.incoming_queue.start()
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            self.write_queue: asyncio.Queue = asyncio.Queue(maxsize=cls.write_queue_size)
            self.write_slots = threading.Semaphore(cls.write_queue_size)
            self.write_task = asyncio.ensure_future(self._drain_write_queue())
            cls.clients_connected += 1
            if cls.client_manager:
                cls.client_manager.add_client(self.client_id, self.request.remote_ip or "")
        except Exception as exc:
            cls.node_handle.get_logger().error(
                f"Unable to accept incoming connection.  Reason: {exc}"
            )

        cls.node_handle.get_logger().info(
            f"Client connected. {cls.clients_connected} clients total."
        )

    @log_exceptions
    def on_message(self, message: str | bytes) -> None:
        if isinstance(message, bytes):
            message = message.decode("utf-8")
        self.incoming_queue.push(message)

    @log_exceptions
    def on_close(self) -> None:
        cls = self.__class__
        assert isinstance(cls.node_handle, Node), "Node handle was not set"
        # Discard outgoing messages to reduce number of failed writes to closed websocket
        # ROS subscriptions have an inherent race condition with protocol.finish()
        self.protocol.outgoing = lambda *_args, **_kwargs: None
        self.incoming_queue.finish()
        self.write_task.cancel()

        cls.clients_connected -= 1
        if cls.client_manager:
            cls.client_manager.remove_client(self.client_id, self.request.remote_ip or "")
        cls.node_handle.get_logger().info(
            f"Client disconnected. {cls.clients_connected} clients total."
        )

    def send_message(self, message: bytes | str, compression: str = "none") -> None:
        cls = self.__class__
        assert isinstance(cls.event_loop, AbstractEventLoop), "Event loop was not set"
        binary = compression in ["cbor", "cbor-raw"]
        if not self.write_slots.acquire(blocking=False):
            assert isinstance(cls.node_handle, Node), "Node handle was not set"
            cls.node_handle.get_logger().warning(
                "Write queue full, dropping outgoing message",
                throttle_duration_sec=1.0,
            )
            return
        cls.event_loop.call_soon_threadsafe(self.write_queue.put_nowait, (message, binary))

    async def _drain_write_queue(self) -> None:
        """Drain the per-connection write queue, serialising all write_message calls."""
        cls = self.__class__
        assert isinstance(cls.node_handle, Node), "Node handle was not set"
        try:
            while True:
                item = await self.write_queue.get()
                self.write_slots.release()
                try:
                    await self.write_message(*item)
                except WebSocketClosedError:
                    cls.node_handle.get_logger().warning(
                        "WebSocketClosedError: Tried to write to a closed websocket",
                        throttle_duration_sec=1.0,
                    )
                    return
                except asyncio.CancelledError:
                    raise
                except Exception:
                    _log_exception()
        except asyncio.CancelledError:
            pass

    @log_exceptions
    def check_origin(self, origin: str) -> bool:  # noqa: ARG002
        return True

    @log_exceptions
    def get_compression_options(self) -> dict | None:
        # If this method returns None (the default), compression will be disabled.
        # If it returns a dict (even an empty one), it will be enabled.
        cls = self.__class__

        if not cls.use_compression:
            return None

        return {}
