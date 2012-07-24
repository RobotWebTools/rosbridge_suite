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

    advertise_msg_fields = [(True, "topic", unicode), (True, "type", unicode)]
    unadvertise_msg_fields = [(True, "topic", unicode)]

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

        # Process the message, catching any exceptions and logging them
        try:
            self._advertise(aid, message)
        except Exception as exc:
            self.protocol.log("error", "advertise: " + exc.message, aid)

    def unadvertise(self, message):
        # Pull out the ID
        aid = message.get("id", None)

        # Process the message, catching any exceptions and logging them
        try:
            self._unadvertise(aid, message)
        except Exception as exc:
            self.protocol.log("error", "unadvertise: " + exc.message, aid)

    def _advertise(self, aid, msg):
        self.basic_type_check(msg, self.advertise_msg_fields)
        topic = msg["topic"]
        msg_type = msg["type"]

        # Create the Registration if one doesn't yet exist
        if not topic in self._registrations:
            client_id = self.protocol.client_id
            self._registrations[topic] = Registration(client_id, topic)

        # Register, propagating any exceptions
        self._registrations[topic].register_advertisement(msg_type, aid)

    def _unadvertise(self, aid, msg):
        self.basic_type_check(msg, self.unadvertise_msg_fields)
        topic = msg["topic"]

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
