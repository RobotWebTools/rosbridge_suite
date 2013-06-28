import importlib
from rosbridge_library.capability import Capability
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

class ReceivedResponses():
    """
    Singleton class to hold lists of received fragments in one 'global' object
    """
    class __impl:
        """ Implementation of the singleton interface """
        def spam(self):
            """ Test method, return singleton id """
            return id(self)

    __instance = None

    list = {}

    def __init__(self):
        """ Create singleton instance """
        if ReceivedResponses.__instance is None:
            ReceivedResponses.__instance = ReceivedResponses.__impl()
            self.list = {}

        self.__dict__['_ReceivedResponses__instance'] = ReceivedResponses.__instance

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)

# service_template could hold a list of "request-objects" that can be identified by request id..
#   these objects could just make the json call to rosbridge-side and wait (blocking) until they receive their answer through a service response
#     that they receive from ServiceServer  (each service request would spawn one of those objects)
#     -> send_request_to_rosbridge-service-provider():   just passes the json-call
#     -> send_response_to_ros-client():                  just returns the response-content /data as if the answer never left ros
# PROBLEM with idea above:
#    -> we register a function in ros whose return gets passed to ros-side
#       ..if not returning from that function we have no way to give back data to the request
# SOLUTION_1:
#    -> use the loop with checking response_list like described in code below
class ROS_Service_Template( threading.Thread):
    service_request_timeout = None

    service_name = None
    service_node_name = None
    service_module = None
    service_type = None
    client_callback = None

    client_id = None
    service_id = None

    request_counter = 0
    request_list = {}     # holds requests until they are answered (response successfully sent to ROS-client)  # probably not needed, but maybe good for retransmission of request or s.th. similar
    response_list = ReceivedResponses().list    # holds service_responses until they are sent back to ROS-client


    def __init__(self, client_callback, service_module, service_type, service_name, client_id):
        threading.Thread.__init__(self)
        
        print "ROS_Service_Template used to create a rosbridge-ServiceInstance"
        self.service_name = service_name
        self.service_module = service_module
        self.service_type = service_type
        self.client_id = client_id
        self.client_callback = client_callback

        #print self.service_name
        #print self.service_type
        

        self.spawn_ROS_service( service_module, service_type, service_name, client_id)

    def handle_service_request(self, req):
        print  "handle_service_request called"

        print "service_request:", req
        print "service_name:", self.service_name
        print "service_type:", self.service_type
        print "client_id:", self.client_id

        print "client_callback:" ,self.client_callback

        #self.client_callback ("requesting")

        # generate request_id
        request_id = self.request_counter
        self.request_counter += 1

        args_list = str(req).split("\n")
        args_dict = {}
        for arg in args_list:
            key, value = arg.split(":")
            args_dict[key] = value

        request_message_object = {"op":"service_request",                                   # advertise topic
        #                            "service_node_name":"nonrosserviceserver",                  #   create python-object
                                    "request_id": request_id,
                                    "service_type": self.service_type,
                                    "service_name": self.service_name,
                                    "args": args_dict
                                    }
        request_message = json.dumps(request_message_object)

        print "request_message:", request_message

        if request_id not in self.request_list.keys():
            self.request_list[request_id] = request_message

        self.client_callback (str(request_message))
        print "sent request to client that serves the service"

#
#        try:
#            exec("from " + self.service_module + " import " + self.service_type+"Response")
#            print "import of",self.service_type+"Response", "from", self.service_module, "succeeded!"
#        except Exception, e:
#            print "import of",self.service_type+"Response", "from", self.service_module, "FAILED!"

        #answer = AddTwoIntsResponse(req.a + req.b)

        # TODO: add timeout to this loop! remove request_id from request_list after timeout!
        while request_id not in self.response_list:
            print "waiting for response to request_id:", request_id
            time.sleep(1)
        print self.response_list[request_id]
        
        answer = self.response_list[request_id]
        del self.response_list[request_id]

        return answer

#        print self.response_list[request_id]
            
        # send JSON request to client that provides this service
        #    add request to request_list and pass id into JSON request
        # loop until timeout or response with request id is received (found in response_list)
        #    when response is found return content/data of response
        #       [return AddTwoIntsResponse(req.a + req.b)]


    def spawn_ROS_service(self, service_module, service_type, service_name, client_id):
        #rospy.init_node(service_node_name)
        print "spawn_ROS_service called"
        try:
            exec("from " + service_module + " import " + service_type)
            print "import of",service_type, "from", service_module, "succeeded!"
        except Exception, e:
            print "import of",service_type, "from", service_module, "FAILED!"

        some_module = importlib.import_module(service_module)
        s = rospy.Service( service_name, getattr(some_module, service_type), self.handle_service_request)
        print "ROS service spawned."
        print "client_id:", self.client_id
        print "service-name:", self.service_name
        print "service-type:", self.service_type
        rospy.spin()

class AdvertiseService(Capability):
    opcode_advertise_service = "advertise_service"      # rosbridge-client -> rosbridge # register in protocol.py!
    opcode_unadvertise_service = "unadvertise_service"      # rosbridge-client -> rosbridge # register in protocol.py!
    opcode_service_response = "service_response"        # rosbridge-client -> rosbridge # register in protocol.py!

    client_opcode_service_request = "service_request"         # rosbridge -> rosbridge-client # not to be registered in protocol.py!

    # have a dict that maps registered services to client id's
    #       service id
    #       client id
    #       service instance ( use a service template that just takes requests and passes them to clients by using methods below)
    #           ..this service instance has to track requests and responses by using a request id that is sent to rosbridge-client and used to identify which response has to be sent to which ros-client
    #           ..this service instance has to use a timeout during which it will wait for an incoming json service_response with correct id from
    service_list = {}

    def __init__(self, protocol):
        self.protocol = protocol
        Capability.__init__(self, protocol)

        protocol.register_operation(self.opcode_advertise_service, self.advertise_service)

    def advertise_service(self, message):
        print "advertise_service called:"
        print "  client_id:", self.protocol.client_id
        # register client internal with service to allow routing of service requests
        print message
        opcode = message["op"]
#        service_node_name = message["service_node_name"]
        service_type = message["service_type"]
        service_name = message["service_name"]
        service_module = message["service_module"]
        client_id = self.protocol.client_id
        client_callback = self.protocol.outgoing
        if service_name not in self.service_list.keys():
            self.service_list[service_name] = ROS_Service_Template(client_callback , service_module, service_type, service_name, client_id)

        #self.service_list[service_name] =
        # register service in ROS


        # have a superclass (or function - see below) that has 'callback' to client for service requests

    # TODO: unadvertise
    def unadvertise_service(self):
        print "unadvertise_service called"
        # register service in ROS

        # have a superclass (or function - see below) that has 'callback' to client for service requests






