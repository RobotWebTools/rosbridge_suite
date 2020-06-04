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

import rospy
import uuid

from rosauth.srv import Authentication

import sys
import threading
import traceback
from functools import partial, wraps

from tornado import version_info as tornado_version_info
from tornado.ioloop import IOLoop
from tornado.iostream import StreamClosedError
from tornado.websocket import WebSocketHandler, WebSocketClosedError
from tornado.gen import coroutine, BadYieldError

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import json, bson


def _log_exception():
    """Log the most recent exception to ROS."""
    exc = traceback.format_exception(*sys.exc_info())
    rospy.logerr(''.join(exc))


def log_exceptions(f):
    """Decorator for logging exceptions to ROS."""
    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except:
            _log_exception()
            raise
    return wrapper


class RosbridgeWebSocket(WebSocketHandler):
    client_id_seed = 0
    clients_connected = 0
    authenticate = False
    use_compression = False

    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = 10 * 1024 * 1024     # bytes
    unregister_timeout = 10.0               # seconds
    bson_only_mode = False

    @log_exceptions
    def open(self):
        rospy.logwarn_once('The Tornado Rosbridge WebSocket implementation is deprecated.'
                           ' See rosbridge_server.autobahn_websocket'
                           ' and rosbridge_websocket.py')
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout,
            "bson_only_mode": cls.bson_only_mode
        }
        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, parameters=parameters)
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            self.authenticated = False
            self._write_lock = threading.RLock()
            cls.client_id_seed += 1
            cls.clients_connected += 1
            self.client_id = uuid.uuid4()
            if cls.client_manager:
                cls.client_manager.add_client(self.client_id, self.request.remote_ip)
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        rospy.loginfo("Client connected.  %d clients total.", cls.clients_connected)
        if cls.authenticate:
            rospy.loginfo("Awaiting proper authentication...")

    @log_exceptions
    def on_message(self, message):
        cls = self.__class__
        # check if we need to authenticate
        if cls.authenticate and not self.authenticated:
            try:
                if cls.bson_only_mode:
                    msg = bson.BSON(message).decode()
                else:
                    msg = json.loads(message)

                if msg['op'] == 'auth':
                    # check the authorization information
                    auth_srv = rospy.ServiceProxy('authenticate', Authentication)
                    resp = auth_srv(msg['mac'], msg['client'], msg['dest'],
                                                  msg['rand'], rospy.Time(msg['t']), msg['level'],
                                                  rospy.Time(msg['end']))
                    self.authenticated = resp.authenticated
                    if self.authenticated:
                        rospy.loginfo("Client %d has authenticated.", self.protocol.client_id)
                        return
                # if we are here, no valid authentication was given
                rospy.logwarn("Client %d did not authenticate. Closing connection.",
                              self.protocol.client_id)
                self.close()
            except:
                # proper error will be handled in the protocol class
                self.protocol.incoming(message)
        else:
            # no authentication required
            self.protocol.incoming(message)

    @log_exceptions
    def on_close(self):
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        if cls.client_manager:
            cls.client_manager.remove_client(self.client_id, self.request.remote_ip)
        rospy.loginfo("Client disconnected. %d clients total.", cls.clients_connected)

    def send_message(self, message):
        if type(message) == bson.BSON:
            binary = True
        elif type(message) == bytearray:
            binary = True
            message = bytes(message)
        else:
            binary = False

        with self._write_lock:
            IOLoop.instance().add_callback(partial(self.prewrite_message, message, binary))

    @coroutine
    def prewrite_message(self, message, binary):
        # Use a try block because the log decorator doesn't cooperate with @coroutine.
        try:
            with self._write_lock:
                future = self.write_message(message, binary)

                # When closing, self.write_message() return None even if it's an undocument output.
                # Consider it as WebSocketClosedError
                # For tornado versions <4.3.0 self.write_message() does not have a return value
                if future is None and tornado_version_info >= (4,3,0,0):
                    raise WebSocketClosedError

                yield future
        except WebSocketClosedError:
            rospy.logwarn_throttle(1, 'WebSocketClosedError: Tried to write to a closed websocket')
            raise
        except StreamClosedError:
            rospy.logwarn_throttle(1, 'StreamClosedError: Tried to write to a closed stream')
            raise
        except BadYieldError:
            # Tornado <4.5.0 doesn't like its own yield and raises BadYieldError.
            # This does not affect functionality, so pass silently only in this case.
            if tornado_version_info < (4, 5, 0, 0):
                pass
            else:
                _log_exception()
                raise
        except:
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
