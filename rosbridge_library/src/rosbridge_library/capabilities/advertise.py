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

from rosbridge_library.capability import Capability
from rosbridge_library.internal.publishers import manager


class Registration():
    """ Keeps track of how many times a client has requested to advertise
    a publisher.

    A client could advertise and unadvertise a topic multiple times, and we
    must make sure that the underlying publisher is only created and destroyed
    at the appropriate moments

    """

    def __init__(self, client_id, topic):
        # Initialise variables
        self.client_id = client_id
        self.topic = topic
        self.clients = {}

    def unregister(self):
        manager.unregister(self.client_id, self.topic)

    def register_advertisement(self, msg_type, adv_id=None):
        # Register with the publisher manager, propagating any exception
        manager.register(self.client_id, self.topic, msg_type)

        self.clients[adv_id] = True

    def unregister_advertisement(self, adv_id=None):
        if adv_id is None:
            self.clients.clear()
        elif adv_id in self.clients:
            del self.clients[adv_id]

    def is_empty(self):
        return len(self.clients) == 0


class Advertise(Capability):

    advertise_msg_fields = [(True, "topic", (str, unicode)), (True, "type", (str, unicode))]
    unadvertise_msg_fields = [(True, "topic", (str, unicode))]

    def __init__(self, protocol):
        # Call superclas constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("advertise", self.advertise)
        protocol.register_operation("unadvertise", self.unadvertise)

        # Initialize class variables
        self._registrations = {}

    def advertise(self, message):
        # Pull out the ID
        aid = message.get("id", None)
        
        self.basic_type_check(message, self.advertise_msg_fields)
        topic = message["topic"]
        msg_type = message["type"]

        # Create the Registration if one doesn't yet exist
        if not topic in self._registrations:
            client_id = self.protocol.client_id
            self._registrations[topic] = Registration(client_id, topic)

        # Register, propagating any exceptions
        self._registrations[topic].register_advertisement(msg_type, aid)

    def unadvertise(self, message):
        # Pull out the ID
        aid = message.get("id", None)

        self.basic_type_check(message, self.unadvertise_msg_fields)
        topic = message["topic"]

        # Now unadvertise the topic
        if topic not in self._registrations:
            return
        self._registrations[topic].unregister_advertisement(aid)

        # Check if the registration is now finished with
        if self._registrations[topic].is_empty():
            self._registrations[topic].unregister()
            del self._registrations[topic]

    def finish(self):
        for registration in self._registrations.values():
            registration.unregister()
        self._registrations.clear()
        self.protocol.unregister_operation("advertise")
        self.protocol.unregister_operation("unadvertise")
