from functools import partial
from rosbridge_library.capability import Capability
from rosbridge_library.internal.services import ServiceCaller


class CallService(Capability):

    call_service_msg_fields = [(True, "service", unicode),
           (False, "fragment_size", int), (False, "compression", unicode)]

    def __init__(self, protocol):
        # Call superclas constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("call_service", self.call_service)

    def call_service(self, message):
        # Pull out the ID
        cid = message.get("id", None)

        # Process the message, catching any exceptions and logging them
        try:
            self._call_service(cid, message)
        except Exception as exc:
            self.protocol.log("error", "call_service: " + exc.message, cid)
            raise

    def _call_service(self, cid, message):
        # Typecheck the args
        self.basic_type_check(message, self.call_service_msg_fields)

        # Create the callbacks
        service = message["service"]
        fragment_size = message.get("fragment_size", None)
        compression = message.get("compression", "none")
        args = message.get("args", [])

        s_cb = partial(self._success, cid, service, fragment_size, compression)
        e_cb = partial(self._failure, cid)

        # Kick off the service caller thread
        ServiceCaller(service, args, s_cb, e_cb).start()

    def _success(self, cid, service, fragment_size, compression, message):
        outgoing_message = {
            "op": "service_response",
            "service": service,
            "values": message
        }
        if cid is not None:
            outgoing_message["cid"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid, exc):
        self.protocol.log("error", "call_service: " + exc.message, cid)
