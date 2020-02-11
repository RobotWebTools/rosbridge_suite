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

from threading import Thread, Condition
from time import time

""" Sits between incoming messages from a subscription, and the outgoing
publish method.  Provides throttling / buffering capabilities.

When the parameters change, the handler may transition to a different kind
of handler
"""


class MessageHandler():
    def __init__(self, previous_handler=None, publish=None):
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

    def set_throttle_rate(self, throttle_rate):
        self.throttle_rate = throttle_rate / 1000.0
        return self.transition()

    def set_queue_length(self, queue_length):
        self.queue_length = queue_length
        return self.transition()

    def time_remaining(self):
        return max((self.last_publish + self.throttle_rate) - time(), 0)

    def handle_message(self, msg):
        self.last_publish = time()
        self.publish(msg)

    def transition(self):
        if self.throttle_rate == 0 and self.queue_length == 0:
            return self
        elif self.queue_length == 0:
            return ThrottleMessageHandler(self)
        else:
            return QueueMessageHandler(self)

    def finish(self):
        pass


class ThrottleMessageHandler(MessageHandler):

    def handle_message(self, msg):
        if self.time_remaining() == 0:
            MessageHandler.handle_message(self, msg)

    def transition(self):
        if self.throttle_rate == 0 and self.queue_length == 0:
            return MessageHandler(self)
        elif self.queue_length == 0:
            return self
        else:
            return QueueMessageHandler(self)

    def finish(self):
        pass


class QueueMessageHandler(MessageHandler, Thread):

    def __init__(self, previous_handler):
        Thread.__init__(self)
        MessageHandler.__init__(self, previous_handler)
        self.daemon = True
        self.queue = []
        self.c = Condition()
        self.alive = True
        self.start()

    def handle_message(self, msg):
        with self.c:
            should_notify = len(self.queue) == 0
            self.queue.append(msg)
            if len(self.queue) > self.queue_length:
                del self.queue[0:len(self.queue) - self.queue_length]
            if should_notify:
                self.c.notify()

    def transition(self):
        if self.throttle_rate == 0 and self.queue_length == 0:
            self.finish()
            return MessageHandler(self)
        elif self.queue_length == 0:
            self.finish()
            return ThrottleMessageHandler(self)
        else:
            with self.c:
                if len(self.queue) > self.queue_length:
                    del self.queue[0:len(self.queue) - self.queue_length]
                self.c.notify()
            return self

    def finish(self):
        """ If throttle was set to 0, this pushes all buffered messages """
        # Notify the thread to finish
        with self.c:
            self.alive = False
            self.c.notify()

        self.join()

    def run(self):
        while self.alive:
            with self.c:
                while self.alive and (self.time_remaining() > 0 or len(self.queue) == 0):
                    if len(self.queue) == 0:
                        self.c.wait()
                    else:
                        self.c.wait(self.time_remaining())
                if self.alive and self.time_remaining() == 0 and len(self.queue) > 0:
                    try:
                        MessageHandler.handle_message(self, self.queue[0])
                    except:
                        pass
                    del self.queue[0]
        while self.time_remaining() == 0 and len(self.queue) > 0:
            try:
                MessageHandler.handle_message(self, self.queue[0])
            except:
                pass
            del self.queue[0]
