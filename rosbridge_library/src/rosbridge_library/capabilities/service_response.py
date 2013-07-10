import importlib
from rosbridge_library.capability import Capability
import rospy
import time
import threading
from advertise_service import ReceivedResponses

# try to import json-lib: 1st try usjon, 2nd try simplejson, else import standard python json
try:
    import ujson as json
    print "using ujson"
except ImportError:
    print "importing ujson failed"
    try:
        import simplejson as json
        print "using simplejson"
    except ImportError:
        print "importing simplejson failed"
        import json
        print "using python default json"


class ServiceResponse(Capability):

    # TODO: unterscheiden von call_service_response
    opcode_service_response = "service_response"        # rosbridge-client -> rosbridge # register in protocol.py!
    response_list = ReceivedResponses().list

    def __init__(self, protocol):
        self.protocol = protocol
        Capability.__init__(self, protocol)

        protocol.register_operation(self.opcode_service_response, self.service_response)


    def service_response(self, message):
        #print "service_response called"
        #print "  ", message
        self.response_list[message["request_id"]] = message["data"]
        # this be the callback for rosbridge-client ; called from within ros-service

        # pass response to ros-side by sending through superclass that provides service to ros-side

    def finish(self):
        self.protocol.unregister_operation("service_response")






