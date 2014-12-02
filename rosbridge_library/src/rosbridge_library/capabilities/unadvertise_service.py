from rosbridge_library.capability import Capability


class UnadvertiseService(Capability):

    #unadvertise_service_msg_fields = [(True, "service", (str, unicode))]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("unadvertise_service", self.unadvertise_service)

    def unadvertise_service(self, message):
        # parse the message
        service_name = message["service"]

        # unregister service in ROS
        if service_name in self.protocol.external_service_list.keys():
            self.protocol.external_service_list[service_name].service_handle.shutdown("Unadvertise request.")
            del self.protocol.external_service_list[service_name]
            self.protocol.log("info", "Unadvertised service %s." % service_name)
        else:
            self.protocol.log("error", "Service %s has no been advertised externally." % service_name)
