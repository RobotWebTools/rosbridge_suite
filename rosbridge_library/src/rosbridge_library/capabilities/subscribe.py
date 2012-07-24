from threading import Lock
from functools import partial
from rospy import loginfo
from rosbridge_library.capability import Capability
from rosbridge_library.internal.subscribers import manager
from rosbridge_library.internal.subscription_modifiers import MessageHandler


class Subscription():
    """ Keeps track of the clients multiple calls to subscribe.

    Chooses the most appropriate settings to send messages """

    def __init__(self, client_id, topic, publish):
        """ Create a subscription for the specified client on the specified
        topic, with callback publish

        Keyword arguments:
        client_id -- the ID of the client making this subscription
        topic     -- the name of the topic to subscribe to
        publish   -- the callback function for incoming messages

        """
        self.client_id = client_id
        self.topic = topic
        self.publish = publish

        self.clients = {}

        self.handler = MessageHandler(None, self._publish)
        self.handler_lock = Lock()
        self.update_params()

    def unregister(self):
        """ Unsubscribes this subscription and cleans up resources """
        manager.unsubscribe(self.client_id, self.topic)
        with self.handler_lock:
            self.handler.finish()
        self.clients.clear()

    def subscribe(self, sid=None, msg_type=None, throttle_rate=0,
                  queue_length=0, fragment_size=None, compression="none"):
        """ Add another client's subscription request

        If there are multiple calls to subscribe, the values actually used for
        queue_length, fragment_size, compression and throttle_rate are
        chosen to encompass all subscriptions' requirements

        Keyword arguments:
        sid             -- the subscription id from the client
        msg_type        -- the type of the message to subscribe to
        throttle_rate   -- the minimum time (in ms) allowed between messages
        being sent.  If multiple subscriptions, the lower of these is used
        queue_length    -- the number of messages that can be buffered.  If
        multiple subscriptions, the lower of these is used
        fragment_size   -- None if no fragmentation, or the maximum length of
        allowed outgoing messages
        compression     -- "none" if no compression, or some other value if
        compression is to be used (current valid values are 'png')

         """
        # Subscribe with the manager. This will propagate any exceptions
        manager.subscribe(self.client_id, self.topic, self.on_msg, msg_type)

        client_details = {
            "throttle_rate": throttle_rate,
            "queue_length": queue_length,
            "fragment_size": fragment_size,
            "compression": compression
        }

        self.clients[sid] = client_details

        self.update_params()

    def unsubscribe(self, sid=None):
        """ Unsubscribe this particular client's subscription

        Keyword arguments:
        sid -- the individual subscription id.  If None, all are unsubscribed

        """
        if sid is None:
            self.clients.clear()
        elif sid in self.clients:
            del self.clients[sid]

        if not self.is_empty():
            self.update_params()

    def is_empty(self):
        """ Return true if there are no subscriptions currently """
        return len(self.clients) == 0

    def _publish(self, message):
        """ Internal method to propagate published messages to the registered
        publish callback """
        self.publish(message, self.fragment_size, self.compression)

    def on_msg(self, msg):
        """ Raw callback called by subscription manager for all incoming
        messages.

        Incoming messages are passed to the message handler which may drop,
        buffer, or propagate the message

        """
        with self.handler_lock:
            self.handler.handle_message(msg)

    def update_params(self):
        """ Determine the 'lowest common denominator' params to satisfy all
        subscribed clients. """
        if len(self.clients) == 0:
            self.throttle_rate = 0
            self.queue_length = 0
            self.fragment_size = None
            self.compression = "none"
            return

        def f(fieldname):
            return [x[fieldname] for x in self.clients.values()]

        self.throttle_rate = min(f("throttle_rate"))
        self.queue_length = min(f("queue_length"))
        frags = [x for x in f("fragment_size") if x != None]
        if frags == []:
            self.fragment_size = None
        else:
            self.fragment_size = min(frags)
        self.compression = "png" if "png" in f("compression") else "none"

        with self.handler_lock:
            self.handler = self.handler.set_throttle_rate(self.throttle_rate)
            self.handler = self.handler.set_queue_length(self.queue_length)


class Subscribe(Capability):

    subscribe_msg_fields = [(True, "topic", unicode), (False, "type", unicode),
        (False, "throttle_rate", int), (False, "fragment_size", int),
        (False, "queue_length", int), (False, "compression", unicode)]
    unsubscribe_msg_fields = [(True, "topic", unicode)]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("subscribe", self.subscribe)
        protocol.register_operation("unsubscribe", self.unsubscribe)

        self._subscriptions = {}

    def subscribe(self, message):
        # Pull out the ID
        sid = message.get("id", None)

        # Process the message, catching any exceptions and logging them
        try:
            self._subscribe(sid, message)
        except Exception as exc:
            self.protocol.log("error", "subscribe: %s" % exc.message, sid)
            raise

    def _subscribe(self, sid, msg):
        # Check the args
        self.basic_type_check(msg, self.subscribe_msg_fields)

        # Make the subscription
        topic = msg["topic"]
        if not topic in self._subscriptions:
            client_id = self.protocol.client_id
            cb = partial(self.publish, topic)
            self._subscriptions[topic] = Subscription(client_id, topic, cb)

        # Register the subscriber
        subscribe_args = {
          "sid": sid,
          "msg_type": msg.get("type", None),
          "throttle_rate": msg.get("throttle_rate", 0),
          "fragment_size": msg.get("fragment_size", None),
          "queue_length": msg.get("queue_length", 0),
          "compression": msg.get("compression", "none")
        }
        self._subscriptions[topic].subscribe(**subscribe_args)

        self.protocol.log("info", "Subscribed to %s" % topic)

    def unsubscribe(self, message):
        # Pull out the ID
        sid = message.get("id", None)

        # Process the message, catching any exceptions and logging them
        try:
            self._unsubscribe(sid, message)
        except Exception as exc:
            self.protocol.log("error", "publish: " + exc.message, sid)
            raise

    def _unsubscribe(self, sid, msg):
        self.basic_type_check(msg, self.unsubscribe_msg_fields)

        topic = msg["topic"]
        if topic not in self._subscriptions:
            return
        self._subscriptions[topic].unsubscribe(sid)

        if self._subscriptions[topic].is_empty():
            self._subscriptions[topic].unregister()
            del self._subscriptions[topic]

        self.protocol.log("info", "Unsubscribed from %s" % topic)

    def publish(self, topic, message, fragment_size=None, compression="none"):
        """ Publish a message to the client

        Keyword arguments:
        topic   -- the topic to publish the message on
        message -- a dict of key-value pairs. Will be wrapped in a message with
        opcode publish
        fragment_size -- (optional) fragment the serialized message into msgs
        with payloads not greater than this value
        compression   -- (optional) compress the message. valid values are
        'png' and 'none'

        """
        # TODO: fragmentation, compression
        outgoing_msg = {"op": "publish", "topic": topic, "msg": message}
        self.protocol.send(outgoing_msg)

    def finish(self):
        for subscription in self._subscriptions.values():
            subscription.unregister()
        self._subscriptions.clear()
        self.protocol.unregister_operation("subscribe")
        self.protocol.unregister_operation("unsubscribe")
