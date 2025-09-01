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

import sys
import time
import traceback
from collections import deque
from threading import Condition, Thread
from typing import TYPE_CHECKING, Generic, TypeVar

if TYPE_CHECKING:
    from collections.abc import Callable

""" Sits between incoming messages from a subscription, and the outgoing
publish method.  Provides throttling / buffering capabilities.

When the parameters change, the handler may transition to a different kind
of handler
"""

MsgT = TypeVar("MsgT")


class MessageHandler(Generic[MsgT]):
    last_publish: float
    throttle_rate: float
    queue_length: int
    publish: Callable[[MsgT], None] | None

    def __init__(
        self,
        previous_handler: MessageHandler | None = None,
        publish: Callable[[MsgT], None] | None = None,
    ) -> None:
        if previous_handler:
            self.last_publish = previous_handler.last_publish
            self.throttle_rate = previous_handler.throttle_rate
            self.queue_length = previous_handler.queue_length
            self.publish = previous_handler.publish
        else:
            self.last_publish = 0
            self.throttle_rate = 0
            self.queue_length = 0
            self.publish = publish

    def set_throttle_rate(self, throttle_rate: float) -> MessageHandler:
        self.throttle_rate = throttle_rate / 1000.0
        return self.transition()

    def set_queue_length(self, queue_length: int) -> MessageHandler:
        self.queue_length = queue_length
        return self.transition()

    def time_remaining(self) -> float:
        return max((self.last_publish + self.throttle_rate) - time.monotonic(), 0)

    def handle_message(self, msg: MsgT) -> None:
        self.last_publish = time.monotonic()
        if self.publish is None:
            err_msg = "No publish method set for MessageHandler"
            raise RuntimeError(err_msg)
        self.publish(msg)

    def transition(self) -> MessageHandler:
        if self.queue_length > 0:
            return QueueMessageHandler(self)
        if self.throttle_rate > 0:
            return ThrottleMessageHandler(self)
        return self

    def finish(self, block: bool = True) -> None:
        pass


class ThrottleMessageHandler(MessageHandler[MsgT]):
    def handle_message(self, msg: MsgT) -> None:
        if self.time_remaining() == 0:
            MessageHandler.handle_message(self, msg)

    def transition(self) -> MessageHandler:
        if self.queue_length > 0:
            return QueueMessageHandler(self)
        if self.throttle_rate > 0:
            return self
        return MessageHandler(self)

    def finish(self, block: bool = True) -> None:
        pass


class QueueMessageHandler(MessageHandler[MsgT], Thread):
    def __init__(self, previous_handler: MessageHandler) -> None:
        Thread.__init__(self)
        MessageHandler.__init__(self, previous_handler)
        self.daemon = True
        self.queue: deque[MsgT] = deque(maxlen=self.queue_length)
        self.c = Condition()
        self.alive = True
        self.start()

    def handle_message(self, msg: MsgT) -> None:
        with self.c:
            if not self.alive:
                return
            should_notify = len(self.queue) == 0
            self.queue.append(msg)
            if should_notify:
                self.c.notify()

    def transition(self) -> MessageHandler:
        if self.queue_length > 0:
            with self.c:
                old_queue = self.queue
                self.queue = deque(maxlen=self.queue_length)
                while len(old_queue) > 0:
                    self.queue.append(old_queue.popleft())
                self.c.notify()
                return self
        self.finish()
        if self.throttle_rate > 0:
            return ThrottleMessageHandler(self)
        return MessageHandler(self)

    def finish(self, block: bool = True) -> None:
        """
        Notify the thread to finish, and optionally wait for it to finish.

        If throttle was set to 0, this pushes all buffered messages.

        :param block: If True, wait for the thread to finish before returning
        """
        # Notify the thread to finish
        with self.c:
            self.alive = False
            self.c.notify()

        if block:
            self.join()

    def run(self) -> None:
        while self.alive:
            msg = None
            with self.c:
                if len(self.queue) == 0:
                    self.c.wait()
                else:
                    self.c.wait(self.time_remaining())
                if self.alive and self.time_remaining() == 0 and len(self.queue) > 0:
                    msg = self.queue.popleft()
            if msg is not None:
                try:
                    MessageHandler.handle_message(self, msg)
                except Exception:
                    traceback.print_exc(file=sys.stderr)
        while self.time_remaining() == 0 and len(self.queue) > 0:
            try:
                msg = self.queue.popleft()
                MessageHandler.handle_message(self, msg)
            except Exception:  # noqa: PERF203
                traceback.print_exc(file=sys.stderr)
