import fnmatch
from threading import Lock
import time

from rosbridge_library.internal.ros_loader import get_service_class
from rosbridge_library.internal import message_conversion
from rosbridge_library.capability import Capability
from rosbridge_library.util import string_types

import rospy

class AdvertisedServiceHandler():

    id_counter = 1
    responses = {}

    def __init__(self, service_name, service_type, protocol):
        self.active_requests = 0
        self.shutdown_requested = False
        self.lock = Lock()
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
        with self.lock:
            self.active_requests += 1
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
            with self.lock:
                if self.shutdown_requested:
                    break
            time.sleep(0)

        with self.lock:
            self.active_requests -= 1

            if self.shutdown_requested:
                self.protocol.log(
                    "warning",
                    "Service %s was unadvertised with a service call in progress, "
                    "aborting service call with request ID %s" % (self.service_name, request_id))
                return None

        resp = self.responses[request_id]
        del self.responses[request_id]
        return resp

    def graceful_shutdown(self, timeout):
        """
        Signal the AdvertisedServiceHandler to shutdown

        Using this, rather than just rospy.Service.shutdown(), allows us
        time to stop any active service requests, ending their busy wait
        loops.
        """
        with self.lock:
            self.shutdown_requested = True
        start_time = time.time()
        while time.time() - start_time < timeout:
            time.sleep(0)

class AdvertiseService(Capability):
    services_glob = None

    advertise_service_msg_fields = [(True, "service", string_types), (True, "type", string_types)]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("advertise_service", self.advertise_service)

    def advertise_service(self, message):
        # Typecheck the args
        self.basic_type_check(message, self.advertise_service_msg_fields)

        # parse the incoming message
        service_name = message["service"]

        if AdvertiseService.services_glob is not None and AdvertiseService.services_glob:
            self.protocol.log("debug", "Service security glob enabled, checking service: " + service_name)
            match = False
            for glob in AdvertiseService.services_glob:
                if (fnmatch.fnmatch(service_name, glob)):
                    self.protocol.log("debug", "Found match with glob " + glob + ", continuing service advertisement...")
                    match = True
                    break
            if not match:
                self.protocol.log("warn", "No match found for service, cancelling service advertisement for: " + service_name)
                return
        else:
            self.protocol.log("debug", "No service security glob, not checking service advertisement.")

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
