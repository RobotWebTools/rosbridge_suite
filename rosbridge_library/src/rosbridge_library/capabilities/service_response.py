from rosbridge_library.capability import Capability
from rosbridge_library.internal import ros_loader, message_conversion
from rosbridge_library.util import string_types


class ServiceResponse(Capability):

    service_response_msg_fields = [
        (True, "service", string_types), (False, "id", string_types),
        (False, "values", dict), (True, "result", bool)
    ]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("service_response", self.service_response)

    def service_response(self, message):
        # Typecheck the args
        self.basic_type_check(message, self.service_response_msg_fields)

        # check for the service
        service_name = message["service"]
        if service_name in self.protocol.external_service_list:
            service_handler = self.protocol.external_service_list[service_name]
            # parse the message
            request_id = message["id"]
            values = message["values"]
            # create a message instance
            resp = ros_loader.get_service_response_instance(service_handler.service_type)
            message_conversion.populate_instance(values, resp)
            # pass along the response
            service_handler.responses[request_id] = resp
        else:
            self.protocol.log("error", "Service %s has not been advertised via rosbridge." % service_name)
