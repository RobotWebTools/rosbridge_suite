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

import fnmatch
from functools import partial
from threading import Lock
from typing import TYPE_CHECKING, Any, Generic

from rosbridge_library.capability import Capability
from rosbridge_library.internal.pngcompression import encode as encode_png
from rosbridge_library.internal.subscribers import manager
from rosbridge_library.internal.subscription_modifiers import MessageHandler
from rosbridge_library.internal.type_support import ROSMessageT

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.node import Node

    from rosbridge_library.internal.outgoing_message import OutgoingMessage
    from rosbridge_library.protocol import Protocol


try:
    from ujson import dumps as encode_json  # type: ignore[import-untyped]
except ImportError:
    try:
        from simplejson import dumps as encode_json  # type: ignore[import-untyped]
    except ImportError:
        from json import dumps as encode_json  # type: ignore[assignment]


class Subscription(Generic[ROSMessageT]):
    """
    Keeps track of the clients multiple calls to subscribe.

    Chooses the most appropriate settings to send messages.
    """

    clients: dict[str, dict[str, Any]]

    def __init__(
        self,
        client_id: str,
        topic: str,
        publish: Callable[[OutgoingMessage[ROSMessageT], int | None, str], None] | None,
        node_handle: Node,
    ) -> None:
        """
        Create a subscription.

        Create a subscription for the specified client on the specified
        topic, with callback publish.

        :param client_id: The ID of the client making this subscription
        :param topic: The name of the topic to subscribe to
        :param publish: The callback function for incoming messages
        :param node_handle: Handle to a rclpy node to create the publisher.
        """
        self.client_id = client_id
        self.topic = topic
        self.publish = publish
        self.node_handle = node_handle

        self.clients = {}

        self.handler: MessageHandler[OutgoingMessage[ROSMessageT]] = MessageHandler(
            None, self._publish
        )
        self.handler_lock = Lock()
        self.update_params()

    def unregister(self) -> None:
        """Unsubscribe this subscription and clean up resources."""
        manager.unsubscribe(self.client_id, self.topic)
        with self.handler_lock:
            self.handler.finish(block=False)
        self.clients.clear()

    def subscribe(
        self,
        sid: str,
        msg_type: str | None = None,
        throttle_rate: int = 0,
        queue_length: int = 0,
        fragment_size: int | None = None,
        compression: str = "none",
    ) -> None:
        """
        Add another client's subscription request.

        If there are multiple calls to subscribe, the values actually used for
        queue_length, fragment_size, compression and throttle_rate are
        chosen to encompass all subscriptions' requirements

        :param sid: The subscription id from the client
        :param msg_type: The type of the message to subscribe to
        :param throttle_rate: The minimum time (in ms) allowed between messages
            being sent. If multiple subscriptions, the lower of these is used
        :param queue_length: The number of messages that can be buffered.  If
            multiple subscriptions, the lower of these is used
        :param fragment_size: None if no fragmentation, or the maximum length of
            allowed outgoing messages
        :param compression: "none" if no compression, or some other value if
            compression is to be used (current valid values are 'png')
        """
        client_details = {
            "throttle_rate": throttle_rate,
            "queue_length": queue_length,
            "fragment_size": fragment_size,
            "compression": compression,
        }

        self.clients[sid] = client_details

        self.update_params()

        raw = compression == "cbor-raw"

        # Subscribe with the manager. This will propagate any exceptions
        manager.subscribe(
            self.client_id,
            self.topic,
            self.on_msg,
            self.node_handle,
            msg_type=msg_type,
            raw=raw,
        )

    def unsubscribe(self, sid: str | None = None) -> None:
        """
        Unsubscribe this particular client's subscription.

        :param sid: The individual subscription id. If None, all are unsubscribed
        """
        if sid is None:
            self.clients.clear()
        elif sid in self.clients:
            del self.clients[sid]

        if not self.is_empty():
            self.update_params()

    def is_empty(self) -> bool:
        """Return True if there are no subscriptions currently."""
        return len(self.clients) == 0

    def _publish(self, message: OutgoingMessage[ROSMessageT]) -> None:
        """
        Publish a message to the subscribed clients.

        Internal method to propagate published messages to the registered
        publish callback.
        """
        if self.publish is not None:
            self.publish(message, self.fragment_size, self.compression)

    def on_msg(self, msg: OutgoingMessage[ROSMessageT]) -> None:
        """
        Handle incoming messages.

        Raw callback called by subscription manager for all incoming
        messages.

        Incoming messages are passed to the message handler which may drop,
        buffer, or propagate the message.
        """
        with self.handler_lock:
            self.handler.handle_message(msg)

    def update_params(self) -> None:
        """
        Update the parameters of the message handler based on current subscriptions.

        Determine the 'lowest common denominator' params to satisfy all
        subscribed clients.
        """
        if len(self.clients) == 0:
            self.throttle_rate = 0
            self.queue_length = 0
            self.fragment_size = None
            self.compression = "none"
            return

        def f(fieldname: str) -> list[Any]:
            return [x[fieldname] for x in self.clients.values()]

        self.throttle_rate = min(f("throttle_rate"))
        self.queue_length = min(f("queue_length"))
        frags = [x for x in f("fragment_size") if x is not None]
        if frags == []:
            self.fragment_size = None
        else:
            self.fragment_size = min(frags)

        self.compression = "none"
        if "png" in f("compression"):
            self.compression = "png"
        if "cbor" in f("compression"):
            self.compression = "cbor"
        if "cbor-raw" in f("compression"):
            self.compression = "cbor-raw"

        with self.handler_lock:
            self.handler.set_throttle_rate(self.throttle_rate)
            self.handler = self.handler.set_queue_length(self.queue_length)


class Subscribe(Capability):
    subscribe_msg_fields = (
        (True, "topic", str),
        (False, "type", str),
        (False, "throttle_rate", int),
        (False, "fragment_size", int),
        (False, "queue_length", int),
        (False, "compression", str),
    )
    unsubscribe_msg_fields = ((True, "topic", str),)

    topics_glob: list[str] | None = None

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("subscribe", self.subscribe)
        protocol.register_operation("unsubscribe", self.unsubscribe)

        self._subscriptions: dict[str, Subscription] = {}

    def subscribe(self, msg: dict[str, Any]) -> None:
        # Pull out the ID
        sid: str | None = msg.get("id")

        # Check the args
        self.basic_type_check(msg, self.subscribe_msg_fields)

        # Make the subscription
        topic: str = msg["topic"]

        if Subscribe.topics_glob is not None and Subscribe.topics_glob:
            self.protocol.log("debug", "Topic security glob enabled, checking topic: " + topic)
            match = False
            for glob in Subscribe.topics_glob:
                if fnmatch.fnmatch(topic, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing subscription...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for topic, cancelling subscription to: " + topic,
                )
                return
        else:
            self.protocol.log("debug", "No topic security glob, not checking subscription.")

        if topic not in self._subscriptions:
            client_id = self.protocol.client_id
            cb = partial(self.publish, topic)
            self._subscriptions[topic] = Subscription(
                client_id, topic, cb, self.protocol.node_handle
            )

        # Register the subscriber
        subscribe_args = {
            "sid": sid,
            "msg_type": msg.get("type"),
            "throttle_rate": msg.get("throttle_rate", 0),
            "fragment_size": msg.get("fragment_size"),
            "queue_length": msg.get("queue_length", 0),
            "compression": msg.get("compression", "none"),
        }
        self._subscriptions[topic].subscribe(**subscribe_args)

        self.protocol.log("info", f"Subscribed to {topic}")

    def unsubscribe(self, msg: dict[str, Any]) -> None:
        # Pull out the ID
        sid: str | None = msg.get("id")

        self.basic_type_check(msg, self.unsubscribe_msg_fields)

        topic: str = msg["topic"]

        if topic not in self._subscriptions:
            return
        self._subscriptions[topic].unsubscribe(sid)

        if self._subscriptions[topic].is_empty():
            self._subscriptions[topic].unregister()
            del self._subscriptions[topic]

        self.protocol.log("info", f"Unsubscribed from {topic}")

    def publish(
        self,
        topic: str,
        message: OutgoingMessage,
        fragment_size: int | None = None,  # noqa: ARG002
        compression: str = "none",
    ) -> None:
        """
        Publish a message to the client.

        :param topic: The topic to publish the message on
        :param message: A ROS message wrapped by OutgoingMessage
        :param fragment_size: (optional) If provided, fragment the serialized message into msgs
            with payloads not greater than this value
        :param compression: (optional) compress the message. valid values are
            'png' and 'none'
        """
        # TODO: fragmentation, proper ids

        outgoing_msg: dict[str, Any] | bytes = {}
        outgoing_msg_raw: dict[str, Any] = {"op": "publish", "topic": topic}
        if compression == "png":
            outgoing_msg_raw["msg"] = message.get_json_values()
            outgoing_msg_dumped: str = encode_json(outgoing_msg_raw)
            outgoing_msg = {"op": "png", "data": encode_png(outgoing_msg_dumped)}
        elif compression == "cbor":
            outgoing_msg = message.get_cbor(outgoing_msg_raw)
        elif compression == "cbor-raw":
            (secs, nsecs) = self.protocol.node_handle.get_clock().now().seconds_nanoseconds()
            outgoing_msg_raw["msg"] = {
                "secs": secs,
                "nsecs": nsecs,
                "bytes": message.message,
            }
            outgoing_msg = message.get_cbor_raw(outgoing_msg_raw)
        else:
            outgoing_msg_raw["msg"] = message.get_json_values()
            outgoing_msg = outgoing_msg_raw

        self.protocol.send(outgoing_msg, compression=compression)

    def finish(self) -> None:
        for subscription in self._subscriptions.values():
            subscription.unregister()
        self._subscriptions.clear()
        self.protocol.unregister_operation("subscribe")
        self.protocol.unregister_operation("unsubscribe")
