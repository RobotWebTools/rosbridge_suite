# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# Copyright (c) 2014, Creativa 77 SRL
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
from typing import TYPE_CHECKING, Any

from rosbridge_library.capabilities.advertise import Registration
from rosbridge_library.capability import Capability
from rosbridge_library.internal.publishers import manager

if TYPE_CHECKING:
    from rosbridge_library.protocol import Protocol


class Publish(Capability):
    publish_msg_fields = (
        (True, "topic", str),
        (False, "type", str),
        (False, "latch", bool),
        (False, "queue_size", int),
        (False, "msg", dict),
    )

    parameter_names = ("topics_glob",)

    topics_glob: list[str] | None = None

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("publish", self.publish)

    def publish(self, message: dict[str, Any]) -> None:
        # Do basic type checking
        self.basic_type_check(message, self.publish_msg_fields)

        # Pull out the ID of the advertisement, if it exists
        adv_id: str | None = message.get("id")

        topic: str = message["topic"]
        msg_type: str | None = message.get("type")
        latch: bool = message.get("latch", False)
        queue_size: int = message.get("queue_size", 100)

        if self.topics_glob is not None:
            self.protocol.log("debug", "Topic security glob enabled, checking topic: " + topic)
            match = False
            for glob in self.topics_glob:
                if fnmatch.fnmatch(topic, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing publish...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn", "No match found for topic, cancelling publish to: " + topic
                )
                return
        else:
            self.protocol.log("debug", "No topic security glob, not checking publish.")

        client_id = self.protocol.client_id

        if topic not in self.protocol.topic_registrations:
            self.protocol.log(
                "info",
                "Trying to publish to unregistered topic: " + topic + ", creating registration...",
            )
            registration = Registration(client_id, topic, self.protocol.node_handle)
            # Register as a publishing client, propagating any exceptions
            registration.register_advertisement(msg_type, adv_id, latch, queue_size)
            self.protocol.topic_registrations[topic] = registration

        # Get the message if one was provided
        msg: dict[str, Any] = message.get("msg", {})

        # Publish the message
        manager.publish(topic, msg)
