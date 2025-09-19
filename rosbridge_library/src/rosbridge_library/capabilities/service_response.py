from __future__ import annotations

from typing import TYPE_CHECKING, Any

from rosbridge_library.capability import Capability
from rosbridge_library.internal import message_conversion, ros_loader

if TYPE_CHECKING:
    from rosbridge_library.protocol import Protocol


class ServiceResponse(Capability):
    service_response_msg_fields = (
        (True, "service", str),
        (False, "id", str),
        (False, "values", dict),
        (True, "result", bool),
    )

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("service_response", self.service_response)

    def service_response(self, message: dict[str, Any]) -> None:
        # Typecheck the args
        self.basic_type_check(message, self.service_response_msg_fields)

        # check for the service
        service_name: str = message["service"]
        if service_name in self.protocol.external_service_list:
            service_handler = self.protocol.external_service_list[service_name]
            # parse the message
            request_id: str = message["id"]
            values: dict[str, Any] = message["values"]
            # create a message instance
            resp = ros_loader.get_service_response_instance(service_handler.service_type)
            message_conversion.populate_instance(values, resp)
            # pass along the response
            service_handler.handle_response(request_id, resp)
        else:
            self.protocol.log(
                "error",
                f"Service {service_name} has not been advertised via rosbridge.",
            )
