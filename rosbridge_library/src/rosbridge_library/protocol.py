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

import time
from typing import TYPE_CHECKING, Any

from rosbridge_library.capabilities.fragmentation import Fragmentation
from rosbridge_library.util import bson, json

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.node import Node

    from rosbridge_library.capabilities.advertise_action import AdvertisedActionHandler
    from rosbridge_library.capabilities.advertise_service import AdvertisedServiceHandler
    from rosbridge_library.capability import Capability


def is_number(s: object) -> bool | None:
    try:
        float(s)  # type: ignore[arg-type]
        return True
    except ValueError:
        return False


def has_binary(obj: object) -> bool:
    """Return True if obj is a binary or contains a binary attribute."""
    if isinstance(obj, list):
        return any(has_binary(item) for item in obj)

    if isinstance(obj, dict):
        return any(has_binary(obj[item]) for item in obj)

    return isinstance(obj, bson.binary.Binary)


class Protocol:
    """
    The interface for a single client to interact with ROS.

    See rosbridge_protocol for the default protocol used by rosbridge

    The lifecycle for a Protocol instance is as follows:
    - Pass incoming messages from the client to incoming
    - Propagate outgoing messages to the client by overriding outgoing
    - Call finish to clean up resources when the client is finished
    """

    # fragment_size can be set per client (each client has its own instance of protocol)
    # ..same for other parameters
    fragment_size = None
    png = None
    # buffer used to gather partial JSON-objects (could be caused by small tcp-buffers or similar..)
    buffer = ""
    old_buffer = ""
    busy = False
    # if this is too low, ("simple")clients network stacks will get flooded (when sending fragments of a huge message..)
    # .. depends on message_size/bandwidth/performance/client_limits/...
    # !! this might be related to (or even be avoided by using) throttle_rate !!
    delay_between_messages = 0
    # global list of non-ros advertised services
    external_service_list: dict[str, AdvertisedServiceHandler]
    # global list of non-ros advertised actions
    external_action_list: dict[str, AdvertisedActionHandler]
    # Use only BSON for the whole communication if the server has been started with bson_only_mode:=True
    bson_only_mode = False

    parameters = None

    def __init__(self, client_id: str, node_handle: Node) -> None:
        """
        Initialize the protocol with a client ID and a ROS2 node handle.

        :param client_id: A unique ID for this client to take. Uniqueness is important, otherwise
            there will be conflicts between multiple clients with shared resources
        :param node_handle: A ROS2 node handle
        """
        self.client_id = client_id
        self.capabilities: list[Capability] = []
        self.operations: dict[str, Callable[[dict[str, Any]], None]] = {}
        self.node_handle = node_handle
        self.external_service_list = {}
        self.external_action_list = {}

        if self.parameters:
            self.fragment_size = self.parameters["max_message_size"]
            self.delay_between_messages = self.parameters["delay_between_messages"]
            self.bson_only_mode = self.parameters.get("bson_only_mode", False)

    # added default message_string="" to allow recalling incoming until buffer is empty without giving a parameter
    # --> allows to get rid of (..or minimize) delay between client-side sends
    def incoming(self, message_string: str = "") -> None:
        """
        Process an incoming message from the client.

        :param message_string: The wire-level message sent by the client
        """
        if len(self.buffer) > 0:
            self.buffer = self.buffer + message_string
        else:
            self.buffer = message_string
        msg = None

        # take care of having multiple JSON-objects in receiving buffer
        # ..first, try to load the whole buffer as a JSON-object
        try:
            msg = self.deserialize(self.buffer)
            self.buffer = ""

        # if loading the whole object fails, try to load a part of it
        # (from first opening bracket "{" to next closing bracket "}")
        # .. this causes Exceptions on "inner" closing brackets --> so I suppressed logging of deserialization errors
        except Exception:
            if self.bson_only_mode:
                # Since BSON should be used in conjunction with a network handler that receives exactly one full BSON
                # message. This will then be passed to self.deserialize and shouldn't cause any exceptions because of
                # fragmented messages (broken or invalid messages might still be sent tough)
                self.log("error", "Exception in deserialization of BSON")

            else:
                # TODO: handling of partial/multiple/broken json data in incoming buffer
                # This way is problematic when json contains nested json-objects
                # ( e.g. { ... { "config": [0,1,2,3] } ...  } )
                # If outer json is not fully received, stepping through opening brackets will find { "config" : ... }
                # as a valid json object and pass this "inner" object to rosbridge and throw away the leading part of
                # the "outer" object. Solution for now: check for "op"-field. I can still imagine cases where a nested
                # message (e.g. complete service_response fits into the data field of a fragment..) would cause trouble,
                # but if a response fits as a whole into a fragment, simply do not pack it into a fragment.
                #
                # --> from that follows current limitation:
                #     fragment data must NOT (!) contain a complete json-object that has an "op-field"
                #
                # An alternative solution would be to only check from first opening bracket and have a time out on data
                # in input buffer (to handle broken data)
                opening_brackets = [i for i, letter in enumerate(self.buffer) if letter == "{"]
                closing_brackets = [i for i, letter in enumerate(self.buffer) if letter == "}"]

                for start in opening_brackets:
                    for end in closing_brackets:
                        try:
                            msg = self.deserialize(self.buffer[start : end + 1])
                            if msg.get("op", None) is not None:
                                # TODO: check if throwing away leading data like this is okay.. loops look okay..
                                self.buffer = self.buffer[end + 1 : len(self.buffer)]
                                # jump out of inner loop if json-decode succeeded
                                break
                        except Exception:
                            # debug json-decode errors with this line
                            # print e
                            pass
                    # if load was successful break outer loop, too.
                    # No need to check if json begins at a "later" opening bracket.
                    if msg is not None:
                        break

        # if decoding of buffer failed .. simply return
        if msg is None:
            return

        # process fields JSON-message object that "control" rosbridge
        mid = None
        if "id" in msg:
            mid = msg["id"]
        if "op" not in msg:
            if "receiver" in msg:
                self.log(
                    "error",
                    "Received a rosbridge v1.0 message. "
                    "Please refer to rosbridge.org for the correct format of rosbridge v2.0 messages. "
                    f"Original message was: {message_string}",
                )
            else:
                self.log(
                    "error",
                    "Received a message without an op. "
                    f"All messages require 'op' field with value one of: {list(self.operations.keys())}. "
                    "Original message was: {message_string}",
                    mid,
                )
            return
        op = msg["op"]
        if op not in self.operations:
            self.log(
                "error",
                f"Unknown operation: {op}.  Allowed operations: {list(self.operations.keys())}",
                mid,
            )
            return
        # This way, a client can change/overwrite its active values anytime by just including parameter field in any
        # message sent to rosbridge. Maybe need to be improved to bind parameter values to specific operation.
        if "fragment_size" in msg:
            self.fragment_size = msg["fragment_size"]
            # print "fragment size set to:", self.fragment_size
        if "message_intervall" in msg and is_number(msg["message_intervall"]):
            self.delay_between_messages = msg["message_intervall"]
        if "png" in msg:
            self.png = msg["msg"]

        # now try to pass message to according operation
        try:
            self.operations[op](msg)
        except Exception as exc:
            self.log("error", f"{op}: {exc!s}", mid)

        # if anything left in buffer, re-call self.incoming
        # TODO: check what happens if we have "garbage" on tcp-stack. Infinite loop might be triggered! Might get out of
        # it when next valid JSON arrives since only data after last 'valid' closing bracket is kept.
        if len(self.buffer) > 0 and self.old_buffer != self.buffer:
            # try to avoid infinite loop..
            self.old_buffer = self.buffer
            self.incoming()

    def outgoing(self, message: bson.BSON | bytearray | str, compression: str = "none") -> None:
        """
        Pass an outgoing message to the client.

        This method should be overridden.

        :param message: The wire-level message to send to the client
        """

    def send(
        self, message: dict[str, Any] | bytes, cid: str | None = None, compression: str = "none"
    ) -> None:
        """
        Prepare a message for sending to the client.

        Called internally in preparation for sending messages to the client.

        This method pre-processes the message then passes it to the overridden
        outgoing method.

        :param message: A dict of message values to be marshalled and sent
        :param cid: (optional) An associated id
        """
        serialized = (
            message if compression in ["cbor", "cbor-raw"] else self.serialize(message, cid)
        )
        if serialized is not None:
            if self.png == "png":
                # TODO: png compression on outgoing messages
                # encode message
                pass

            fragment_list = None
            if self.fragment_size is not None and len(serialized) > self.fragment_size:
                mid = None
                if isinstance(message, dict) and "id" in message:
                    mid = message["id"]

                # TODO: think about splitting into fragments that have specified size including header-fields!
                # --> estimate header size --> split content into fragments that have the requested overall size,
                # rather than requested content size
                fragment_list = Fragmentation(self).fragment(message, self.fragment_size, mid)

            # fragment list not empty -> send fragments
            if fragment_list is not None:
                for fragment in fragment_list:
                    if self.bson_only_mode:
                        self.outgoing(bson.BSON.encode(fragment), compression)
                    else:
                        self.outgoing(json.dumps(fragment), compression)
                    # okay to use delay here (sender's send()-function) because rosbridge is sending next request only
                    # to service provider when last one had finished. If this was not the case, this delay would need to
                    # be implemented in service-provider's (meaning message receiver's) send_message()-function in
                    # rosbridge_tcp.py)
                    time.sleep(self.delay_between_messages)
            # else send message as it is
            else:
                self.outgoing(serialized, compression)
                time.sleep(self.delay_between_messages)

    def finish(self) -> None:
        """
        Indicate that the client is finished and clean up resources.

        All clients should call this method after disconnecting.
        """
        for capability in self.capabilities:
            capability.finish()

    def serialize(
        self,
        msg: bytearray | bson.BSON | dict[str, Any],
        cid: str | None = None,  # noqa: ARG002
    ) -> bson.BSON | bytearray | str | None:
        """
        Turn a dictionary of values into the appropriate wire-level representation.

        Default behaviour uses JSON. Override to use a different container.

        :param msg: The dictionary of values to serialize
        :param cid: (optional) An ID associated with this. Will be logged on err.

        :return: a JSON string representing the dictionary
        """
        try:
            if isinstance(msg, bytearray):
                return msg
            if has_binary(msg) or self.bson_only_mode:
                return bson.BSON.encode(msg)
            return json.dumps(msg)
        except Exception as e:
            self.log("error", f"Unable to serialize message '{msg}': {e}")
            return None

    def deserialize(self, msg: str | bytes | bytearray, cid: str | None = None) -> dict[str, Any]:  # noqa: ARG002
        """
        Turn the wire-level representation into a dictionary of values.

        Default behaviour assumes JSON. Override to use a different container.

        :param msg: The wire-level message to deserialize
        :param cid: (optional) An ID associated with this. Is logged on error

        :return: a dictionary of values
        """
        try:
            if self.bson_only_mode:
                bson_message = bson.BSON(msg)
                return bson_message.decode()
            return json.loads(msg)
        except Exception:
            # if we did try to deserialize the whole buffer, first try to let self.incoming check for multiple/partial
            # json-decodes before logging error. This means, if buffer is not == msg --> we tried to decode part of
            # the buffer.

            # TODO: implement a way to have a final Exception when nothing works out to decode
            # (multiple/broken/partial JSON..)

            # suppressed logging of exception on json-decode to keep rosbridge-logs "clean",
            # otherwise console logs would get spammed for every failed json-decode try
            #            if msg != self.buffer:
            #                error_msg = "Unable to deserialize message from client: %s"  % msg
            #                error_msg += "\nException was: " +str(e)
            #
            #                self.log("error", error_msg, cid)

            # re-raise Exception to allow handling outside of deserialize function instead of returning None
            raise
            # return None

    def register_operation(self, opcode: str, handler: Callable[[dict[str, Any]], None]) -> None:
        """
        Register a handler for an opcode.

        :param opcode: The opcode to register this handler for
        :param handler: A callback function to call for messages with this opcode
        """
        self.operations[opcode] = handler

    def unregister_operation(self, opcode: str) -> None:
        """
        Unregister a handler for an opcode.

        :param opcode: The opcode to unregister the handler for
        """
        if opcode in self.operations:
            del self.operations[opcode]

    def add_capability(self, capability_class: type[Capability]) -> None:
        """
        Add a capability to the protocol.

        This method is for convenience; assumes the default capability constructor.

        :param capability_class: The class of the capability to add
        """
        self.capabilities.append(capability_class(self))

    def log(self, level: str, message: str, lid: str | None = None) -> None:
        """
        Log a message to the client.

        By default just sends the message to the node logger.

        :param level: The logger level of this message
        :param message: The string message to send to the user
        :param lid: An associated id for this log message
        """
        stdout_formatted_msg = None
        stdout_formatted_msg = (
            f"[Client {self.client_id}] [id: {lid}] {message}"
            if lid is not None
            else f"[Client {self.client_id}] {message}"
        )

        if level in {"error", "err"}:
            self.node_handle.get_logger().error(stdout_formatted_msg)
        elif level in {"warning", "warn"}:
            self.node_handle.get_logger().warning(stdout_formatted_msg)
        elif level in {"info", "information"}:
            self.node_handle.get_logger().info(stdout_formatted_msg)
        else:
            self.node_handle.get_logger().debug(stdout_formatted_msg)
