from rosbridge_library.capability import Capability
from advertise_service import ServiceList


class StopService(Capability):
    opcode_unadvertise_service = "stop_service"

    service_list = ServiceList().list

    def __init__(self, protocol):
        self.protocol = protocol 
        Capability.__init__(self, protocol)
        protocol.register_operation(self.opcode_unadvertise_service, self.unadvertise_service)

    def unadvertise_service(self, message):
        service_name = message["service_name"]
        client_id = self.protocol.client_id

        # unregister service in ROS
        if service_name in self.service_list.keys():
            if client_id == self.service_list[service_name].client_id:
                self.service_list[service_name].stop_ROS_service()
            else:
                self.protocol.log("warning", "tried to remove service: " + service_name + " ['owned' by client: " + str(self.service_list[service_name].client_id) + "]")
        else:
            self.protocol.log("warning", "tried to remove service: " + service_name + " [service was not registered]")

    def finish(self):
        self.protocol.unregister_operation("unadvertise_server")
    







