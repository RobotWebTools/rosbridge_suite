import importlib
from rosbridge_library.capability import Capability
from advertise_service import ServiceList
import rospy
import time
import threading

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



class StopService(Capability):
    opcode_unadvertise_service = "stop_service"      # rosbridge-client -> rosbridge # register in protocol.py!


    # have a dict that maps registered services to client id's
    #       service id
    #       client id
    #       service instance ( use a service template that just takes requests and passes them to clients by using methods below)
    #           ..this service instance has to track requests and responses by using a request id that is sent to rosbridge-client and used to identify which response has to be sent to which ros-client
    #           ..this service instance has to use a timeout during which it will wait for an incoming json service_response with correct id from
    service_list = ServiceList().list

    def __init__(self, protocol):
        self.protocol = protocol 
        Capability.__init__(self, protocol)

        protocol.register_operation(self.opcode_unadvertise_service, self.unadvertise_service)

    # TODO: unadvertise
    def unadvertise_service(self, message):
        print "unadvertise_service called"
        opcode = message["op"]
        service_name = message["service_name"]
        print " service name:", service_name
        print " self.service_list:", self.service_list
        
        client_id = self.protocol.client_id

        # unregister service in ROS
        if service_name in self.service_list.keys():
            print " service found"
            if client_id == self.service_list[service_name].client_id:
                print "  requesting client_id matches providing client_id"
                self.service_list[service_name].stop_ROS_service()
                print " ROS service stopped"
                del self.service_list[service_name]
                print "rosbridge service removed"
        else:
            print " service not found!"


        # have a superclass (or function - see below) that has 'callback' to client for service requests






