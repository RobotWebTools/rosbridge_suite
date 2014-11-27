from rosbridge_library.internal.ros_loader import get_service_class
from rosbridge_library.internal import message_conversion
from rosbridge_library.capability import Capability
import rospy
import time


class AdvertisedServiceHandler():

    id_counter = 1
    responses = {}

    def __init__(self, service_name, service_type, protocol):
        self.service_name = service_name
        self.service_type = service_type
        self.protocol = protocol
        # setup the service
        self.service_handle = rospy.Service(service_name, get_service_class(service_type), self.handle_request)

    def next_id(self):
        id = self.id_counter
        self.id_counter += 1
        return id

    def handle_request(self, req):
        # generate a unique ID
        request_id = "service_request:" + self.service_name + ":" + str(self.next_id())

        # build a request to send to the external client
        request_message = {
            "op": "call_service",
            "id": request_id,
            "service": self.service_name,
            "args": message_conversion.extract_values(req)
        }
        self.protocol.send(request_message)

        # wait for a response
        while request_id not in self.responses.keys():
            time.sleep(0)

        resp = self.responses[request_id]
        del self.responses[request_id]
        return resp


class AdvertiseService(Capability):

    advertise_service_msg_fields = [(True, "service", (str, unicode)), (True, "type", (str, unicode))]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("advertise_service", self.advertise_service)

    def advertise_service(self, message):
        # parse the incoming message
        service_name = message["service"]

        # check for an existing entry
        if service_name in self.protocol.external_service_list.keys():
            self.protocol.log("warn", "Duplicate service advertised. Overwriting %s." % service_name)
            self.protocol.external_service_list[service_name].service_handle.shutdown("Duplicate advertiser.")
            del self.protocol.external_service_list[service_name]

        # setup and store the service information
        service_type = message["type"]
        service_handler = AdvertisedServiceHandler(service_name, service_type, self.protocol)
        self.protocol.external_service_list[service_name] = service_handler
        self.protocol.log("info", "Advertised service %s." % service_name)
