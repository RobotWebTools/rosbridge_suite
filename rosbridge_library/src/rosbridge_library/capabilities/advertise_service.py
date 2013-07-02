import importlib
from rosbridge_library.capability import Capability
import rospy
from datetime import datetime
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

class ServiceList():
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
        if ServiceList.__instance is None:
            ServiceList.__instance = ServiceList.__impl()
            self.list = {}

        self.__dict__['_ServiceList__instance'] = ServiceList.__instance

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)

    

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
    service_request_timeout = 2 #seconds
    check_response_delay = 0.5 #seconds

    service_name = None
    service_node_name = None
    service_module = None
    service_type = None
    client_callback = None

    client_id = None
    service_id = None

    ros_serviceproxy = None

    request_counter = 0
    request_list = {}     # holds requests until they are answered (response successfully sent to ROS-client)  # probably not needed, but maybe good for retransmission of request or s.th. similar
    response_list = ReceivedResponses().list    # holds service_responses until they are sent back to ROS-client


    def __init__(self, client_callback, service_module, service_type, service_name, client_id):
        threading.Thread.__init__(self)
        
        print " ROS_Service_Template used to create a rosbridge-ServiceInstance"
        self.service_name = service_name
        self.service_module = service_module
        self.service_type = service_type
        self.client_id = client_id
        self.client_callback = client_callback

        #print self.service_name
        #print self.service_type
        
        self.spawn_ROS_service( service_module, service_type, service_name, client_id)

    def handle_service_request(self, req):
        print "----------------------------------------------------------------"
        print  "handle_service_request called"

        print "  service_request:"
        print req
        print "  service_name:", self.service_name
        print "  service_type:", self.service_type
        print "  service providing client_id:", self.client_id

        print "  client_callback:" ,self.client_callback

        #self.client_callback ("requesting")

        # generate request_id
        request_id = self.request_counter
        self.request_counter += 1   # TODO modulo blabla..

        # TODO: check for more complex parameter and types and bla --> need better parser!
        # --> see message_conversion
        args_list = str(req).split("\n")
        args_dict = {}
        for arg in args_list:
            key, value = arg.split(":")
            args_dict[key] = value

        request_message_object = {"op":"service_request",                                   # advertise topic
                                    "request_id": request_id,
                                    "service_type": self.service_type,
                                    "service_name": self.service_name,
                                    "args": args_dict
                                    }
        request_message = json.dumps(request_message_object)

        print " request_message:", request_message

        # TODO: check cases! this cond should not be necessary
        if request_id not in self.request_list.keys():
            self.request_list[request_id] = request_message

        self.client_callback (str(request_message))
        print " sent request to client that serves the service"

#
#        try:
#            exec("from " + self.service_module + " import " + self.service_type+"Response")
#            print "import of",self.service_type+"Response", "from", self.service_module, "succeeded!"
#        except Exception, e:
#            print "import of",self.service_type+"Response", "from", self.service_module, "FAILED!"

        #answer = AddTwoIntsResponse(req.a + req.b)

        # TODO: add timeout to this loop! remove request_id from request_list after timeout!
        begin = datetime.now()
        duration = datetime.now() - begin
        answer = None
        while request_id not in self.response_list and duration.total_seconds() < self.service_request_timeout:
            print " waiting for response to request_id:", request_id
            time.sleep(self.check_response_delay)
            duration = datetime.now() - begin

        if request_id in self.response_list:

            print "  response_list:", self.response_list
            print "  request_list:", self.request_list

            answer = self.response_list[request_id]
            del self.response_list[request_id]


        else:
            # request failed due to timeout
            print "request timed out!"

            answer = None

        del self.request_list[request_id]
        print "----------------------------------------------------------------"
        return answer
#        print self.response_list[request_id]
            
        # send JSON request to client that provides this service
        #    add request to request_list and pass id into JSON request
        # loop until timeout or response with request id is received (found in response_list)
        #    when response is found return content/data of response
        #       [return AddTwoIntsResponse(req.a + req.b)]

    def stop_ROS_service(self):
        print " stopping ROS service"
        self.ros_serviceproxy.shutdown("reason: stop service requested")
        


    def spawn_ROS_service(self, service_module, service_type, service_name, client_id):
        #rospy.init_node(service_node_name)
        print " spawn_ROS_service called"
        try:
            exec("from " + service_module + " import " + service_type)
            print "  import of",service_type, "from", service_module, "succeeded!"
        except Exception, e:
            print "  import of",service_type, "from", service_module, "FAILED!"

        some_module = importlib.import_module(service_module)
        self.ros_serviceproxy = rospy.Service( service_name, getattr(some_module, service_type), self.handle_service_request)
        print " ROS service spawned."
        print "  client_id:", self.client_id
        print "  service-name:", self.service_name
        print "  service-type:", self.service_type
        #rospy.spin()






class AdvertiseService(Capability):
    opcode_advertise_service = "advertise_service"      # rosbridge-client -> rosbridge # register in protocol.py!

    service_list = ServiceList().list

    def __init__(self, protocol):
        self.protocol = protocol
        Capability.__init__(self, protocol)

        protocol.register_operation(self.opcode_advertise_service, self.advertise_service)

    def advertise_service(self, message):
        print "advertise_service called:"
        print "  client_id:", self.protocol.client_id
        # register client internal with service to allow routing of service requests
        print "  ", message
        opcode = message["op"]
        service_type = message["service_type"]
        service_name = message["service_name"]
        service_module = message["service_module"]
        client_id = self.protocol.client_id
        client_callback = self.protocol.outgoing
        # TODO: define what happens when existing service gets advertised
        if service_name not in self.service_list.keys():
            print " registering new service, did not exist before.."
            self.service_list[service_name] = ROS_Service_Template(client_callback , service_module, service_type, service_name, client_id)
        else:
            print " replacing existing service"
            self.service_list[service_name].stop_ROS_service()
            del self.service_list[service_name]
            self.service_list[service_name] = ROS_Service_Template(client_callback , service_module, service_type, service_name, client_id)

        print "  self.service_list:", self.service_list
        #self.service_list[service_name] =
        # register service in ROS


        # have a superclass (or function - see below) that has 'callback' to client for service requests








