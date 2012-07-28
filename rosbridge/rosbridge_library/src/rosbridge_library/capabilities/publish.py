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
        # Pull out the ID
        pid = message.get("id", None)

        # Process the message, catching any exceptions and logging them
        try:
            self._publish(message)
        except Exception as exc:
            self.protocol.log("error", "publish %s: %s" %
            (exc.__class__.__name__, exc.message), pid)
            raise

    def _publish(self, message):
        self.basic_type_check(message, self.publish_msg_fields)
        topic = message["topic"]

        # Register as a publishing client, propagating any exceptions
        client_id = self.protocol.client_id
        manager.register(client_id, topic)
        self._published[topic] = True

        # Get the message if one was provided
        msg = message.get("msg", {})

        manager.publish(client_id, topic, msg)

    def finish(self):
        client_id = self.protocol.client_id
        for topic in self._published:
            manager.unregister(client_id, topic)
        self._published.clear()
