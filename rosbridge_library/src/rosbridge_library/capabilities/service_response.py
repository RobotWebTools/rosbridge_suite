from rosbridge_library.capability import Capability
from advertise_service import ReceivedResponses

# try to import json-lib: 1st try usjon, 2nd try simplejson, else import standard python json
try:
    import ujson as json
except ImportError:
    try:
        import simplejson as json
    except ImportError:
        import json



class ServiceResponse(Capability):

    # this defines an opcode for incoming responses (seen from rosbridge, response coming from non-ros service provider connected to rosbridge)
    #  call_service uses the same opcode for sending responses to non-ros clients that requested a service..
    #  this is works fine, but feel free to change opcode-names to whatever you think suits better..
    opcode_service_response = "service_response"        # rosbridge-client -> rosbridge # register in protocol.py!
    response_list = ReceivedResponses().list

    def __init__(self, protocol):
        self.protocol = protocol
        Capability.__init__(self, protocol)

        protocol.register_operation(self.opcode_service_response, self.service_response)


    def service_response(self, message):
        # just put response into singleton list for responses.. the requesting process will find it there and clean up after delivery to client
        self.response_list[message["request_id"]] = message["data"]


    def finish(self):
        self.protocol.unregister_operation("service_response")






