# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# Copyright (c) 2013, PAL Robotics SL
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

from threading import Lock, RLock

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosbridge_library.internal import ros_loader
from rosbridge_library.internal.message_conversion import msg_class_type_repr
from rosbridge_library.internal.outgoing_message import OutgoingMessage
from rosbridge_library.internal.topics import (
    TopicNotEstablishedException,
    TypeConflictException,
)

""" Manages and interfaces with ROS Subscriber objects.  A single subscriber
is shared between multiple clients
"""


class MultiSubscriber:
    """Handles multiple clients for a single subscriber.

    Converts msgs to JSON before handing them to callbacks.  Due to subscriber
    callbacks being called in separate threads, must lock whenever modifying
    or accessing the subscribed clients."""

    def __init__(self, topic, client_id, callback, node_handle, msg_type=None, raw=False):
        """Register a subscriber on the specified topic.

        Keyword arguments:
        topic    -- the name of the topic to register the subscriber on
        client_id -- the ID of the client subscribing
        callback  -- this client's callback, that will be called for incoming
        messages
        node_handle -- Handle to a rclpy node to create the publisher.
        msg_type -- (optional) the type to register the subscriber as.  If not
        provided, an attempt will be made to infer the topic type

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

        # Certain combinations of publisher and subscriber QoS parameters are
        # incompatible. Here we make a "best effort" attempt to match existing
        # publishers for the requested topic. This is not perfect because more
        # publishers may come online after our subscriber is set up, but we try
        # to provide sane defaults.
        # For this reason we use volatile durability and best effort reliability
        # to prioritize topic compatibility when the publisher policy is not known.
        # For more information, see:
        # - https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
        # - https://github.com/RobotWebTools/rosbridge_suite/issues/551
        # - https://github.com/RobotWebTools/rosbridge_suite/issues/769
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        infos = node_handle.get_publishers_info_by_topic(topic)

        if len(infos) > 0 and all(
            pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for pub in infos
        ):
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
        if any(pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for pub in infos):
            qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Create the subscriber and associated member variables
        # Subscriptions is initialized with the current client to start with.
        self.subscriptions = {client_id: callback}
        self.rlock = RLock()
        self.msg_class = msg_class
        self.node_handle = node_handle
        self.topic = topic
        self.qos = qos
        self.raw = raw
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.subscriber = node_handle.create_subscription(
            msg_class, topic, self.callback, qos, raw=raw, callback_group=self.callback_group
        )
        self.new_subscriber = None
        self.new_subscriptions = {}

    def unregister(self):
        self.node_handle.destroy_subscription(self.subscriber)
        with self.rlock:
            self.subscriptions.clear()
            if self.new_subscriber:
                self.node_handle.destroy_subscription(self.new_subscriber)
                self.new_subscriber = None

    def verify_type(self, msg_type):
        """Verify that the subscriber subscribes to messages of this type.

        Keyword arguments:
        msg_type -- the type to check this subscriber against

        Throws:
        Exception -- if ros_loader cannot load the specified msg type
        TypeConflictException -- if the msg_type is different than the type of
        this publisher

        """
        if not ros_loader.get_message_class(msg_type) is self.msg_class:
            raise TypeConflictException(self.topic, msg_class_type_repr(self.msg_class), msg_type)

    def subscribe(self, client_id, callback):
        """Subscribe the specified client to this subscriber.

        Keyword arguments:
        client_id -- the ID of the client subscribing
        callback  -- this client's callback, that will be called for incoming
        messages

        """
        with self.rlock:
            # If the topic is latched, adding a new subscriber will immediately invoke
            # the given callback.
            # In any case, the first message is handled using new_sub_callback,
            # which adds the new callback to the subscriptions dictionary.
            self.new_subscriptions.update({client_id: callback})
            infos = self.node_handle.get_publishers_info_by_topic(self.topic)

            if len(infos) > 0 and all(
                pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for pub in infos
            ):
                self.qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            if any(pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for pub in infos):
                self.qos.reliability = ReliabilityPolicy.BEST_EFFORT

            if self.new_subscriber is None:
                self.new_subscriber = self.node_handle.create_subscription(
                    self.msg_class,
                    self.topic,
                    self._new_sub_callback,
                    self.qos,
                    raw=self.raw,
                    callback_group=self.callback_group,
                )

    def unsubscribe(self, client_id):
        """Unsubscribe the specified client from this subscriber

        Keyword arguments:
        client_id -- the ID of the client to unsubscribe

        """
        with self.rlock:
            if client_id in self.new_subscriptions:
                del self.new_subscriptions[client_id]
            if client_id in self.subscriptions:
                del self.subscriptions[client_id]

    def has_subscribers(self):
        """Return true if there are subscribers"""
        with self.rlock:
            return len(self.subscriptions) + len(self.new_subscriptions) != 0

    def callback(self, msg, callbacks=None):
        """Callback for incoming messages on the rclpy subscription.

        Passes the message to registered subscriber callbacks.

        Keyword Arguments:
        msg - the ROS message coming from the subscriber
        callbacks - subscriber callbacks to invoke

        """
        outgoing = OutgoingMessage(msg)

        with self.rlock:
            callbacks = callbacks or self.subscriptions.values()

            # Pass the JSON to each of the callbacks
            for callback in callbacks:
                try:
                    callback(outgoing)
                except Exception as exc:
                    # Do nothing if one particular callback fails except log it
                    self.node_handle.get_logger().error(
                        f"Exception calling subscribe callback: {exc}"
                    )

    def _new_sub_callback(self, msg):
        """
        Callbacks for new subscribers.

        If the topic was latched, a new subscriber has to be added to receive
        a new message and route it to the new subscriptor.

        After the first message is routed, the new subscriber is deleted and
        the subscriptions dictionary is updated with the newly incorporated
        subscriptors.
        """
        with self.rlock:
            self.callback(msg, self.new_subscriptions.values())
            self.subscriptions.update(self.new_subscriptions)
            self.new_subscriptions = {}
            self.node_handle.destroy_subscription(self.new_subscriber)
            self.new_subscriber = None


class SubscriberManager:
    """
    Keeps track of client subscriptions
    """

    def __init__(self):
        self._lock = Lock()
        self._subscribers = {}

    def subscribe(self, client_id, topic, callback, node_handle, msg_type=None, raw=False):
        """Subscribe to a topic

        Keyword arguments:
        client_id -- the ID of the client making this subscribe request
        topic     -- the name of the topic to subscribe to
        callback  -- the callback to call for incoming messages on the topic
        msg_type  -- (optional) the type of the topic

        """
        with self._lock:
            if topic not in self._subscribers:
                self._subscribers[topic] = MultiSubscriber(
                    topic, client_id, callback, node_handle, msg_type=msg_type, raw=raw
                )
            else:
                self._subscribers[topic].subscribe(client_id, callback)

            if msg_type is not None and not raw:
                self._subscribers[topic].verify_type(msg_type)

    def unsubscribe(self, client_id, topic):
        """Unsubscribe from a topic

        Keyword arguments:
        client_id -- the ID of the client to unsubscribe
        topic     -- the topic to unsubscribe from

        """
        with self._lock:
            if topic not in self._subscribers:
                return

            self._subscribers[topic].unsubscribe(client_id)

            if not self._subscribers[topic].has_subscribers():
                self._subscribers[topic].unregister()
                del self._subscribers[topic]


manager = SubscriberManager()
