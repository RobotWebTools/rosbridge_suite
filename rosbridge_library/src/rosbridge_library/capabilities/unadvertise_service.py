from __future__ import annotations

import fnmatch
from typing import TYPE_CHECKING, Any

from rosbridge_library.capability import Capability

if TYPE_CHECKING:
    from rosbridge_library.protocol import Protocol


class UnadvertiseService(Capability):
    unadvertise_service_msg_fields = ((True, "service", str),)

    parameter_names = ("services_glob",)

    services_glob: list[str] | None = None

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("unadvertise_service", self.unadvertise_service)

    def unadvertise_service(self, message: dict[str, Any]) -> None:
        self.basic_type_check(message, self.unadvertise_service_msg_fields)

        # parse the message
        service_name: str = message["service"]

        if self.services_glob is not None:
            self.protocol.log(
                "debug",
                "Service security glob enabled, checking service: " + service_name,
            )
            match = False
            for glob in self.services_glob:
                if fnmatch.fnmatch(service_name, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing service unadvertisement...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for service, cancelling service unadvertisement for: "
                    + service_name,
                )
                return
        else:
            self.protocol.log(
                "debug",
                "No service security glob, not checking service unadvertisement...",
            )

        # unregister service in ROS
        if service_name in self.protocol.external_service_list:
            self.protocol.external_service_list[service_name].graceful_shutdown()
            self.protocol.external_service_list[service_name].service_handle.destroy()
            del self.protocol.external_service_list[service_name]
            self.protocol.log("info", f"Unadvertised service {service_name}")
        else:
            self.protocol.log(
                "error",
                f"Service {service_name} has not been advertised via rosbridge, can't unadvertise.",
            )
