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

from threading import Timer

from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosbridge_library.internal import message_conversion, ros_loader
from rosbridge_library.internal.message_conversion import msg_class_type_repr
from rosbridge_library.internal.topics import (
    TopicNotEstablishedException,
    TypeConflictException,
)


class MultiPublisher:
    """Keeps track of the clients that are using a particular publisher.

    Provides an API to publish messages and register clients that are using
    this publisher"""

    def __init__(self, topic, node_handle, msg_type=None, latched_client_id=None, queue_size=100):
        """Register a publisher on the specified topic.

        Keyword arguments:
        topic    -- the name of the topic to register the publisher to
        node_handle -- Handle to a rclpy node to create the publisher.
        msg_type -- (optional) the type to register the publisher as.  If not
        provided, an attempt will be made to infer the topic type
        latch    -- (optional) if a client requested this publisher to be latched,
                    provide the client_id of that client here

        Throws:
        TopicNotEstablishedException -- if no msg_type was specified by the
        caller and the topic is not yet established, so a topic type cannot
        be inferred
        TypeConflictException        -- if the msg_type was specified by the
        caller and the topic is established, and the established type is
        different to the user-specified msg_type

        """
        # First check to see if the topic is already established
        topics_names_and_types = dict(node_handle.get_topic_names_and_types())
        topic_type = topics_names_and_types.get(topic)

        # If it's not established and no type was specified, exception
        if msg_type is None and topic_type is None:
            raise TopicNotEstablishedException(topic)

        # topic_type is a list of types or None at this point; only one type is supported.
        if topic_type is not None:
            if len(topic_type) > 1:
                node_handle.get_logger().warning(f"More than one topic type detected: {topic_type}")
            topic_type = topic_type[0]

        # Use the established topic type if none was specified
        if msg_type is None:
            msg_type = topic_type

        # Load the message class, propagating any exceptions from bad msg types
        msg_class = ros_loader.get_message_class(msg_type)

        # Make sure the specified msg type and established msg type are same
        msg_type_string = msg_class_type_repr(msg_class)
        if topic_type is not None and topic_type != msg_type_string:
            raise TypeConflictException(topic, topic_type, msg_type_string)

        # Create the publisher and associated member variables
        self.clients = {}
        self.latched_client_id = latched_client_id
        self.topic = topic
        self.node_handle = node_handle
        self.msg_class = msg_class
        # Adding a lifespan solves the problem of late-joining subscribers
        # without the need of a custom message publisher implementation.
        publisher_qos = QoSProfile(
            depth=queue_size,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # For latched clients, no lifespan has to be specified (i.e. latch forever).
        # Otherwise we want to keep the messages for a second to prevent late-joining subscribers from
        # missing messages.
        if latched_client_id is None:
            publisher_qos.lifespan = Duration(seconds=1)
        else:
            publisher_qos.depth = 1

        self.publisher = node_handle.create_publisher(msg_class, topic, qos_profile=publisher_qos)

    def unregister(self):
        """Unregisters the publisher and clears the clients"""
        self.node_handle.destroy_publisher(self.publisher)
        self.clients.clear()

    def verify_type(self, msg_type):
        """Verify that the publisher publishes messages of the specified type.

        Keyword arguments:
        msg_type -- the type to check this publisher against

        Throws:
        Exception -- if ros_loader cannot load the specified msg type
        TypeConflictException -- if the msg_type is different than the type of
        this publisher

        """
        if not ros_loader.get_message_class(msg_type) is self.msg_class:
            raise TypeConflictException(self.topic, msg_class_type_repr(self.msg_class), msg_type)
        return

    def publish(self, msg):
        """Publish a message using this publisher.

        Keyword arguments:
        msg -- the dict (json) message to publish

        Throws:
        Exception -- propagates exceptions from message conversion if the
        provided msg does not properly conform to the message type of this
        publisher

        """
        # Create a message instance
        inst = self.msg_class()

        # Populate the instance, propagating any exceptions that may be thrown
        message_conversion.populate_instance(msg, inst, self.node_handle.get_clock())

        # Publish the message
        self.publisher.publish(inst)

    def register_client(self, client_id):
        """Register the specified client as a client of this publisher.

        Keyword arguments:
        client_id -- the ID of the client using the publisher

        """
        self.clients[client_id] = True

    def unregister_client(self, client_id):
        """Unregister the specified client from this publisher.

        If the specified client_id is not a client of this publisher, nothing
        happens.

        Keyword arguments:
        client_id -- the ID of the client to remove

        """
        if client_id in self.clients:
            del self.clients[client_id]

    def has_clients(self):
        """Return true if there are clients to this publisher."""
        return len(self.clients) != 0


class PublisherManager:
    """The PublisherManager keeps track of ROS publishers

    It maintains a MultiPublisher instance for each registered topic

    When unregistering a client, if there are no more clients for a publisher,
    then that publisher is unregistered from the ROS Master
    """

    def __init__(self):
        self._publishers = {}
        self.unregister_timers = {}
        self.unregister_timeout = 10.0

    def register(self, client_id, topic, node_handle, msg_type=None, latch=False, queue_size=100):
        """Register a publisher on the specified topic.

        Publishers are shared between clients, so a single MultiPublisher
        instance is created per topic, even if multiple clients register.

        Keyword arguments:
        client_id  -- the ID of the client making this request
        topic      -- the name of the topic to publish on
        node_handle -- Handle to a rclpy node to create the publisher.
        msg_type   -- (optional) the type to publish
        latch      -- (optional) whether to make this publisher latched
        queue_size -- (optional) publisher queue_size to use

        Throws:
        Exception -- exceptions are propagated from the MultiPublisher if
        there is a problem loading the specified msg class or establishing
        the publisher

        """
        latched_client_id = client_id if latch else None
        if topic not in self._publishers:
            self._publishers[topic] = MultiPublisher(
                topic,
                node_handle,
                msg_type=msg_type,
                latched_client_id=latched_client_id,
                queue_size=queue_size,
            )
        elif latch and self._publishers[topic].latched_client_id != client_id:
            node_handle.get_logger().warn(
                f"Client ID {client_id} attempted to register topic [{topic}] as "
                "latched but this topic was previously registered."
            )
            node_handle.get_logger().warn(
                "Only a single registered latched publisher is supported at the time"
            )
        elif not latch and self._publishers[topic].latched_client_id:
            node_handle.get_logger().warn(
                f"New non-latched publisher registration for topic [{topic}] which is "
                "already registered as latched. but this topic was previously registered."
            )
            node_handle.get_logger().warn(
                "Only a single registered latched publisher is supported at the time"
            )

        if msg_type is not None:
            self._publishers[topic].verify_type(msg_type)

        self._publishers[topic].register_client(client_id)

    def unregister(self, client_id, topic):
        """Unregister a client from the publisher for the given topic.
            Will wait some time before actually unregistering, it is done in
            _unregister_impl

        If there are no clients remaining for that publisher, then the
        publisher is unregistered from the ROS Master

        Keyword arguments:
        client_id -- the ID of the client making this request
        topic     -- the topic to unregister the publisher for

        """
        if topic not in self._publishers:
            return

        self._publishers[topic].unregister_client(client_id)
        if topic in self.unregister_timers:
            self.unregister_timers[topic].cancel()
            del self.unregister_timers[topic]
        self.unregister_timers[topic] = Timer(
            self.unregister_timeout, self._unregister_impl, [topic]
        )
        self.unregister_timers[topic].start()

    def _unregister_impl(self, topic):
        if not self._publishers[topic].has_clients():
            self._publishers[topic].unregister()
            del self._publishers[topic]
        del self.unregister_timers[topic]

    def unregister_all(self, client_id):
        """Unregisters a client from all publishers that they are registered
        to.

        Keyword arguments:
        client_id -- the ID of the client making this request"""
        for topic in self._publishers.keys():
            self.unregister(client_id, topic)

    def publish(self, client_id, topic, msg, node_handle, latch=False, queue_size=100):
        """Publish a message on the given topic.

        Tries to create a publisher on the topic if one does not already exist.

        Keyword arguments:
        client_id -- the ID of the client making this request
        topic     -- the topic to publish the message on
        msg       -- a JSON-like dict of fields and values
        node_handle -- Handle to a rclpy node to create the publisher.
        latch     -- (optional) whether to make this publisher latched
        queue_size -- (optional) publisher queue_size to use

        Throws:
        Exception -- a variety of exceptions are propagated.  They can be
        thrown if there is a problem setting up or getting the publisher,
        or if the provided msg does not map to the msg class of the publisher.

        """
        self.register(client_id, topic, node_handle, latch=latch, queue_size=queue_size)

        self._publishers[topic].publish(msg)


manager = PublisherManager()
