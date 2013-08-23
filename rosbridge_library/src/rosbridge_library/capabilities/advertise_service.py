from rosbridge_library.internal.ros_loader import get_service_class
from rosbridge_library.internal import message_conversion

from rosbridge_library.capability import Capability
import rospy
from datetime import datetime
import time
import threading



class ServiceList():
    """
    Singleton class to hold lists of registered services in one 'global' object
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

class RequestList():
    """
    Singleton class to hold lists of received requests in one 'global' object
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
        if RequestList.__instance is None:
            RequestList.__instance = RequestList.__impl()
            self.list = {}

        self.__dict__['_RequestList__instance'] = RequestList.__instance

    def __getattr__(self, attr):
        """ Delegate access to implementation """
        return getattr(self.__instance, attr)

    def __setattr__(self, attr, value):
        """ Delegate access to implementation """
        return setattr(self.__instance, attr, value)


class ReceivedResponses():
    """
    Singleton class to hold lists of received service responses in one 'global' object
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


# instances of this class are created for every externally advertised service
class ROS_Service_Template( threading.Thread):
    service_request_timeout = 600           #seconds
    check_response_delay = 0.1              #seconds
    wait_for_busy_service_provider = 0.1    #seconds

    max_requests = 500000

    service_name = None
    service_node_name = None
    service_module = None
    service_type = None
    client_callback = None

    client_id = None
    service_id = None

    ros_serviceproxy = None

    request_counter = 0

    response_list = ReceivedResponses().list    # holds service_responses until they are sent back to ROS-client
    protocol = None

    finish_flag = False
    busy = False
    spawned = False

    def __init__(self, client_callback, service_module, service_type, service_name, client_id, protocol):
        threading.Thread.__init__(self)

        self.protocol = protocol
        self.service_name = service_name
        self.service_module = service_module
        self.service_type = service_type
        self.client_id = client_id
        self.client_callback = client_callback

        # populate parameters
        if self.protocol.parameters != None:
            self.service_request_timeout  = self.protocol.parameters["service_request_timeout"]
            self.check_response_delay = self.protocol.parameters["check_response_delay"]
            self.wait_for_busy_service_provider = self.protocol.parameters["wait_for_busy_service_provider"]
            self.max_requests = self.protocol.parameters["max_service_requests"]

        self.spawn_ROS_service( service_module, service_type, service_name, client_id)


    def handle_service_request(self, req):
        # stay in this loop until (this) instance is not waiting for response for an "old" request
        # (.. probably not best solution so far)
        while not self.spawned or self.busy:
            # if stop_service was called.. 
            if self.finish_flag:
                # kill unsent requests to that service
                return None
            time.sleep(self.wait_for_busy_service_provider)

        self.busy = True

        # generate request_id
        # ..service_name avoids having same id's for different service-requests
        request_id = "service:" + self.service_name + "_count:" + str(self.request_counter) + "_time:" + datetime.now().strftime("%H:%M:%f")
        # increment request_counter
        self.request_counter = (self.request_counter + 1) % self.max_requests

        req_extracted = message_conversion.extract_values(req)
        request_message_object = {"op":"service_request",
                                  "request_id": request_id,
                                  "args": req_extracted
                                 }
                                    
        # add request to request_list
        if request_id not in self.protocol.request_list.keys():
            # put information about request into request_list, we need this later to create a response instance with service-module and -type
            self.protocol.request_list[request_id] = { "service_name" : self.service_name,
                                                       "service_module" : self.service_module,
                                                       "service_type" : self.service_type
                                                     }
        # answer will be passed to client that requested service
        answer = None
        try:
            # send JSON-service request to client that is providing the service (via protocol)
            self.client_callback( request_message_object )
            begin = datetime.now()
            duration = datetime.now() - begin
            # wait for service response by checking response_list
            # ..if stop_service was called.. stop waiting for response
            while not self.finish_flag and request_id not in self.response_list.keys() and duration.total_seconds() < self.service_request_timeout:
                time.sleep(self.check_response_delay)
                duration = datetime.now() - begin
            # if response found..
            if request_id in self.response_list:
                answer = self.response_list[request_id]
                # remove response from response_list
                del self.response_list[request_id]
            else:
                # request failed due to timeout (or some other reason?!)
                print "request timed out!"
                answer = None
            # remove request from request_list
            del self.protocol.request_list[request_id]
        # TODO: more detailed exception handling
        except Exception, e:
            print e
        self.busy = False
        return answer

    def stop_ROS_service(self):
        self.spawned = False
        self.finish_flag = True
        self.ros_serviceproxy.shutdown("reason: stop_service requested by providing client")
        # set answer for unfinished requests to None
        # --> response can be found, but will be the same as for failed/timed out requests..
        for request_id in self.protocol.request_list.keys():
            self.protocol.response_list[request_id] = None
        # wait for request_loops to run into finish_flags
        while len(self.protocol.request_list) > 0:
            time.sleep(self.check_response_delay)
        # remove service from service_list
#        service_list = ServiceList().list
        del self.protocol.service_list[self.service_name]
        self.protocol.log("info", "removed service: "+ self.service_name)
    
    def spawn_ROS_service(self, service_module, service_type, service_name, client_id):
        # import service type for ros-service that we want to register in ros
        service_class = get_service_class(service_module+'/'+service_type)
        # register service in ros
        self.ros_serviceproxy = rospy.Service( service_name, service_class, self.handle_service_request)

        # TODO: check if service successfully registered in ros. current state is that rosbridge "thinks" service is registered, but calls will fail
#        myServiceManager = rospy.service.ServiceManager()
#        myServiceManager.register(service_name, self.ros_serviceproxy)
#        print myServiceManager.get_services()

        # log service registration
        log_msg = "registered service: " + self.service_name
        self.protocol.log("info", log_msg)
        self.spawned = True

class AdvertiseService(Capability):

    opcode_advertise_service = "advertise_service"      # rosbridge-client -> rosbridge # register in protocol.py!
    service_list = ServiceList().list                   # links to singleton
    request_list = RequestList().list     # holds requests until they are answered (response successfully sent to ROS-client)

    def __init__(self, protocol):
        self.protocol = protocol
        Capability.__init__(self, protocol)
        protocol.register_operation(self.opcode_advertise_service, self.advertise_service)
        self.protocol.service_list = self.service_list
        self.protocol.request_list = self.request_list

    def advertise_service(self, message):
        opcode = message["op"]
        service_type = message["service_type"]
        service_name = message["service_name"]
        service_module = message["service_module"]
        client_id = self.protocol.client_id
        client_callback = self.protocol.send
        # this part defines what is happening when a client is trying to "replace" an already registered service
        if service_name not in self.service_list.keys():
            self.service_list[service_name] = ROS_Service_Template(client_callback , service_module, service_type, service_name, client_id, self.protocol)
        else:
            log_msg = "is replacing service: " + service_name + " [provided before by client:" + str(self.service_list[service_name].client_id) + "]"
            self.protocol.log("info", log_msg)
            self.service_list[service_name].stop_ROS_service()
            self.service_list[service_name] = ROS_Service_Template(client_callback , service_module, service_type, service_name, client_id, self.protocol )

    def finish(self):
        self.finish_flag = True
        self.protocol.unregister_operation("advertise_service")
