from rosbridge_library.capability import Capability
from advertise_service import ReceivedResponses
from rosbridge_library.internal import ros_loader, message_conversion


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
        # convert the message into according response instance
        # then just put response into singleton list for responses.. the requesting process will find it there and clean up after delivery to client

        # get information about the request with the same id as the incoming response
        # ..this information gets written into "request_list" by advertise_service.py within "handle_service_request()"
        request = self.protocol.request_list[message["request_id"]]

        # get module and type
        service_module = request["service_module"]
        service_type = request["service_type"]

        ## Create a message instance
        inst = ros_loader.get_service_response_instance(service_module+"/"+service_type)
        
        # Populate the instance, propagating any exceptions that may be thrown
        message_conversion.populate_instance(message["data"], inst)

        # add response instance to response_list
        self.response_list[message["request_id"]] = inst


    def finish(self):
        self.protocol.unregister_operation("service_response")






