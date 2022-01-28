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

import fnmatch

from rosbridge_library.capability import Capability
from rosbridge_library.internal.publishers import manager


class Publish(Capability):

    publish_msg_fields = [(True, "topic", str)]

    topics_glob = None

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("publish", self.publish)

        # Save the topics that are published on for the purposes of unregistering
        self._published = {}

        if protocol.parameters and "unregister_timeout" in protocol.parameters:
            manager.unregister_timeout = protocol.parameters.get("unregister_timeout")

    def publish(self, message):
        # Do basic type checking
        self.basic_type_check(message, self.publish_msg_fields)
        topic = message["topic"]
        latch = message.get("latch", False)
        queue_size = message.get("queue_size", 100)

        if Publish.topics_glob is not None and Publish.topics_glob:
            self.protocol.log("debug", "Topic security glob enabled, checking topic: " + topic)
            match = False
            for glob in Publish.topics_glob:
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

        # Register as a publishing client, propagating any exceptions
        client_id = self.protocol.client_id
        manager.register(
            client_id,
            topic,
            self.protocol.node_handle,
            latch=latch,
            queue_size=queue_size,
        )
        self._published[topic] = True

        # Get the message if one was provided
        msg = message.get("msg", {})

        # Publish the message
        manager.publish(
            client_id,
            topic,
            msg,
            self.protocol.node_handle,
            latch=latch,
            queue_size=queue_size,
        )

    def finish(self):
        client_id = self.protocol.client_id
        for topic in self._published:
            manager.unregister(client_id, topic)
        self._published.clear()
