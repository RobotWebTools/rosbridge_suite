# Software License Agreement (BSD License)
#
# Copyright (c) 2025, Fictionlab sp. z o.o.
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

from rclpy.node import Node
from rclpy.task import Future


async def futures_wait_for(node: Node, futures: list[Future], timeout_sec: float):
    """Await a list of futures with a timeout."""
    first_done_future: Future = Future()

    def timeout_callback():
        first_done_future.set_result(None)

    timer = node.create_timer(timeout_sec, timeout_callback)

    def future_done_callback(arg):
        if all(future.done() for future in futures):
            first_done_future.set_result(None)

    for future in futures:
        future.add_done_callback(future_done_callback)

    await first_done_future

    timer.cancel()
    timer.destroy()


async def async_sleep(node: Node, delay_sec: float):
    """Block the coroutine for a given time."""
    sleep_future: Future = Future()

    def timeout_callback():
        sleep_future.set_result(None)

    timer = node.create_timer(delay_sec, timeout_callback)

    await sleep_future

    timer.cancel()
    timer.destroy()
