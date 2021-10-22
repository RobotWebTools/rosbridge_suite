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

import sys
import threading
import traceback
import uuid
from collections import deque
from functools import partial, wraps

import rclpy
from rosbridge_msgs.srv import HttpAuthentication
from rosbridge_msgs.msg import HttpHeaderField
from rosbridge_library.internal.services import ServiceCaller
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import bson
from tornado import version_info as tornado_version_info
from tornado.gen import BadYieldError, coroutine
from tornado.ioloop import IOLoop
from tornado.iostream import StreamClosedError
from tornado.websocket import WebSocketClosedError, WebSocketHandler

from typing import (Any)

_io_loop = IOLoop.instance()


def _log_exception():
    """Log the most recent exception to ROS."""
    exc = traceback.format_exception(*sys.exc_info())
    RosbridgeWebSocket.node_handle.get_logger().error("".join(exc))


def log_exceptions(f):
    """Decorator for logging exceptions to ROS."""

    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except Exception:
            _log_exception()
            raise

    return wrapper


class IncomingQueue(threading.Thread):
    """Decouples incoming messages from the Tornado thread.

    This mitigates cases where outgoing messages are blocked by incoming,
    and vice versa.
    """

    def __init__(self, protocol):
        threading.Thread.__init__(self)
        self.daemon = True
        self.queue = deque()
        self.protocol = protocol

        self.cond = threading.Condition()
        self._finished = False

    def finish(self):
        """Clear the queue and do not accept further messages."""
        with self.cond:
            self._finished = True
            while len(self.queue) > 0:
                self.queue.popleft()
            self.cond.notify()

    def push(self, msg):
        with self.cond:
            self.queue.append(msg)
            self.cond.notify()

    def run(self):
        while True:
            with self.cond:
                if len(self.queue) == 0 and not self._finished:
                    self.cond.wait()

                if self._finished:
                    break

                msg = self.queue.popleft()

            self.protocol.incoming(msg)

        self.protocol.finish()


class RosbridgeWebSocket(WebSocketHandler):
    clients_connected = 0
    use_compression = False

    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600  # seconds
    # protocol.py:
    delay_between_messages = 0  # seconds
    max_message_size = 10000000  # bytes
    unregister_timeout = 10.0  # seconds
    bson_only_mode = False
    node_handle = None

    # Optional service name that is called to authenticate each client
    # connection
    authentication_service = None

    @log_exceptions
    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        cls = self.__class__
        if self.authentication_service is not None:
            self.auth_client = cls.node_handle.create_client(HttpAuthentication, self.authentication_service)
            if not self.auth_client.wait_for_service(timeout_sec=5.0):
                cls.node_handle.get_logger().warn('Authentication service %s not available'
                                                   % self.authentication_service)


    @log_exceptions
    async def get(self, *args: Any, **kwargs: Any) -> None:
        cls = self.__class__
        self.client_id = uuid.uuid4()
        (allowed,
         auth_response_code,
         auth_response_headers,
         log_msg) = self.check_authentication()
        if auth_response_headers is not None:
            for field in auth_response_headers:
                self.set_header(field.name, field.value)
        if allowed:
            super().get(*args, **kwargs)
        else:
            self.set_status(auth_response_code or 403)
            log_msg = "Authorization required"
            self.finish(log_msg)

    @log_exceptions
    def open(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout,
            "bson_only_mode": cls.bson_only_mode,
        }
        try:
            self.protocol = RosbridgeProtocol(
                self.client_id, cls.node_handle, parameters=parameters
            )
            self.incoming_queue = IncomingQueue(self.protocol)
            self.incoming_queue.start()
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            self._write_lock = threading.RLock()
            cls.clients_connected += 1
            if cls.client_manager:
                cls.client_manager.add_client(self.client_id, self.request.remote_ip)
        except Exception as exc:
            cls.node_handle.get_logger().error(
                f"Unable to accept incoming connection.  Reason: {exc}"
            )

        cls.node_handle.get_logger().info(
            f"Client connected. {cls.clients_connected} clients total."
        )


    @log_exceptions
    def check_authentication(self) -> (bool, int, [], str):
        cls = self.__class__
        if self.auth_client is None:
            return (true, 200, None, None)
        allowed = False
        auth_req = HttpAuthentication.Request()
        auth_req.client_connection_id = str(self.client_id)
        h = self.request.headers
        for (k,v) in sorted(h.get_all()):
            auth_req.headers.append(HttpHeaderField(name=k, value=v))
        try:
            auth_future = self.auth_client.call_async(auth_req)
            rclpy.spin_until_future_complete(cls.node_handle, auth_future, timeout_sec=5.0)
        except Exception as e:
            cls.node_handle.get_logger().error('Service call failed %r' % (e,))
            return (false, 500, None, "Authentication service call failed")
        else:
            if not auth_future.done():
                cls.node_handle.get_logger().error('Service call timed out while waiting for response from %s'
                                                   % self.authentication_service)
                return (false, 500, None, "Authentication service call timed out")
            auth_response = auth_future.result()
            allowed = auth_response.authenticated
            auth_response_headers = auth_response.headers
            auth_response_code = auth_response.status_code
            have_server_error = False
            server_error_msg = "Authentication service call failed."
            if auth_response.client_connection_id:
                # Set the client ID provided by the authentication service
                self.client_id = auth_response.client_connection_id
                # Check an upper bound on the client_id length
                if len(auth_response.client_connection_id) > 4000:
                    have_server_error = True
                    server_error_msg += " Bad client_id response: %s" \
                        % (auth_response.client_connection_id)
            if allowed and auth_response_code and auth_response_code != 200:
                have_server_error = True
                server_error_msg += " Bad status_code: %s" % auth_response_code
            if have_server_error:
                return (false, 500, None, server_error_msg)
            else:
                return (allowed, auth_response_code, auth_response_headers, None)


    @log_exceptions
    def on_message(self, message):
        if isinstance(message, bytes):
            message = message.decode("utf-8")
        self.incoming_queue.push(message)

    @log_exceptions
    def on_close(self):
        cls = self.__class__
        cls.clients_connected -= 1
        if cls.client_manager:
            cls.client_manager.remove_client(self.client_id, self.request.remote_ip)
        cls.node_handle.get_logger().info(
            f"Client disconnected. {cls.clients_connected} clients total."
        )
        self.incoming_queue.finish()

    def send_message(self, message):
        if isinstance(message, bson.BSON):
            binary = True
        elif isinstance(message, bytearray):
            binary = True
            message = bytes(message)
        else:
            binary = False

        with self._write_lock:
            _io_loop.add_callback(partial(self.prewrite_message, message, binary))

    @coroutine
    def prewrite_message(self, message, binary):
        cls = self.__class__
        # Use a try block because the log decorator doesn't cooperate with @coroutine.
        try:
            with self._write_lock:
                future = self.write_message(message, binary)

                # When closing, self.write_message() return None even if it's an undocument output.
                # Consider it as WebSocketClosedError
                # For tornado versions <4.3.0 self.write_message() does not have a return value
                if future is None and tornado_version_info >= (4, 3, 0, 0):
                    raise WebSocketClosedError

                yield future
        except WebSocketClosedError:
            cls.node_handle.get_logger().warn(
                "WebSocketClosedError: Tried to write to a closed websocket",
                throttle_duration_sec=1.0,
            )
            raise
        except StreamClosedError:
            cls.node_handle.get_logger().warn(
                "StreamClosedError: Tried to write to a closed stream",
                throttle_duration_sec=1.0,
            )
            raise
        except BadYieldError:
            # Tornado <4.5.0 doesn't like its own yield and raises BadYieldError.
            # This does not affect functionality, so pass silently only in this case.
            if tornado_version_info < (4, 5, 0, 0):
                pass
            else:
                _log_exception()
                raise
        except:  # noqa: E722  # Will log and raise
            _log_exception()
            raise

    @log_exceptions
    def check_origin(self, origin):
        return True

    @log_exceptions
    def get_compression_options(self):
        # If this method returns None (the default), compression will be disabled.
        # If it returns a dict (even an empty one), it will be enabled.
        cls = self.__class__

        if not cls.use_compression:
            return None

        return {}
