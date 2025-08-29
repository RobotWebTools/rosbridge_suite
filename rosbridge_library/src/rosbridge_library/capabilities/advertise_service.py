from __future__ import annotations

import fnmatch
from typing import TYPE_CHECKING, Any, Generic

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from rosbridge_library.capability import Capability
from rosbridge_library.internal import message_conversion
from rosbridge_library.internal.ros_loader import get_service_class
from rosbridge_library.internal.type_support import (
    ROSMessage,
    ROSServiceRequestT,
    ROSServiceResponseT,
)

if TYPE_CHECKING:
    from rclpy.service import Service

    from rosbridge_library.protocol import Protocol


class AdvertisedServiceHandler(Generic[ROSServiceRequestT, ROSServiceResponseT]):
    id_counter = 1

    def __init__(self, service_name: str, service_type: str, protocol: Protocol) -> None:
        self.request_futures: dict[str, Future[ROSServiceResponseT]] = {}
        self.service_name = service_name
        self.service_type = service_type
        self.protocol = protocol
        # setup the service
        self.service_handle: Service[ROSServiceRequestT, ROSServiceResponseT] = (
            protocol.node_handle.create_service(
                get_service_class(service_type),
                service_name,
                self.handle_request,  # type: ignore[arg-type]  # rclpy type hint does not support coroutines
                callback_group=ReentrantCallbackGroup(),  # https://github.com/ros2/rclpy/issues/834#issuecomment-961331870
            )
        )

    def next_id(self) -> int:
        next_id_value = self.id_counter
        self.id_counter += 1
        return next_id_value

    async def handle_request(
        self, req: ROSServiceRequestT, _res: ROSServiceResponseT
    ) -> ROSServiceResponseT:
        # generate a unique ID
        request_id = f"service_request:{self.service_name}:{self.next_id()}"

        future: Future[ROSServiceResponseT] = Future()
        self.request_futures[request_id] = future

        # build a request to send to the external client
        request_message = {
            "op": "call_service",
            "id": request_id,
            "service": self.service_name,
            "args": message_conversion.extract_values(req),
        }
        self.protocol.send(request_message)

        try:
            result = await future
            assert result is not None, "Service response cannot be None"
            return result
        finally:
            del self.request_futures[request_id]

    def handle_response(self, request_id: str, res: ROSServiceResponseT) -> None:
        """
        Handle service response.

        Called by the ServiceResponse capability to handle a service response from the external client.
        """
        if request_id in self.request_futures:
            self.request_futures[request_id].set_result(res)
        else:
            self.protocol.log(
                "warning", f"Received service response for unrecognized id: {request_id}"
            )

    def graceful_shutdown(self) -> None:
        """
        Signal the AdvertisedServiceHandler to shutdown.

        Using this, rather than just node_handle.destroy_service, allows us
        time to stop any active service requests, ending their busy wait
        loops.
        """
        if self.request_futures:
            incomplete_ids = ", ".join(self.request_futures.keys())
            self.protocol.log(
                "warning",
                f"Service {self.service_name} was unadvertised with a service call in progress, "
                f"aborting service calls with request IDs {incomplete_ids}",
            )
            for future_id in self.request_futures:
                future = self.request_futures[future_id]
                future.set_exception(RuntimeError(f"Service {self.service_name} was unadvertised"))
        self.service_handle.destroy()


class AdvertiseService(Capability):
    services_glob: list[str] | None = None

    advertise_service_msg_fields = ((True, "service", str), (True, "type", str))

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("advertise_service", self.advertise_service)

    def advertise_service(self, message: dict[str, Any]) -> None:
        # Typecheck the args
        self.basic_type_check(message, self.advertise_service_msg_fields)

        # parse the incoming message
        service_name: str = message["service"]

        if AdvertiseService.services_glob is not None and AdvertiseService.services_glob:
            self.protocol.log(
                "debug",
                "Service security glob enabled, checking service: " + service_name,
            )
            match = False
            for glob in AdvertiseService.services_glob:
                if fnmatch.fnmatch(service_name, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing service advertisement...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for service, cancelling service advertisement for: "
                    + service_name,
                )
                return
        else:
            self.protocol.log(
                "debug", "No service security glob, not checking service advertisement."
            )

        # check for an existing entry
        if service_name in self.protocol.external_service_list:
            self.protocol.log("warn", f"Duplicate service advertised. Overwriting {service_name}.")
            self.protocol.external_service_list[service_name].graceful_shutdown()
            del self.protocol.external_service_list[service_name]

        # setup and store the service information
        service_type: str = message["type"]
        service_handler: AdvertisedServiceHandler[ROSMessage, ROSMessage] = (
            AdvertisedServiceHandler(service_name, service_type, self.protocol)
        )
        self.protocol.external_service_list[service_name] = service_handler
        self.protocol.log("info", f"Advertised service {service_name}")
