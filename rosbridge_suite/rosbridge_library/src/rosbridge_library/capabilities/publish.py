from rosbridge_library.capability import Capability
from rosbridge_library.internal.publishers import manager


class Publish(Capability):

    publish_msg_fields = [(True, "topic", unicode)]

    def __init__(self, protocol):
        # Call superclas constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("publish", self.publish)
        
        # Save the topics that are published on for the purposes of unregistering
        self._published = {}

    def publish(self, message):
        # Do basic type checking
        self.basic_type_check(message, self.publish_msg_fields)
        topic = message["topic"]

        # Register as a publishing client, propagating any exceptions
        client_id = self.protocol.client_id
        manager.register(client_id, topic)
        self._published[topic] = True

        # Get the message if one was provided
        msg = message.get("msg", {})

        # Publish the message
        manager.publish(client_id, topic, msg)
        
    def finish(self):
        client_id = self.protocol.client_id
        for topic in self._published:
            manager.unregister(client_id, topic)
        self._published.clear()
