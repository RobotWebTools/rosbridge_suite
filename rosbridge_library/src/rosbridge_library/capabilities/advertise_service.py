from rosbridge_library.internal.ros_loader import get_service_class
from rosbridge_library.internal.message_conversion import extract_values

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

import pprint

class DictDotLookup(object):
    """
    Creates objects that behave much like a dictionaries, but allow nested
    key access using object '.' (dot) lookups.
    """
    def __init__(self, d):
        for k in d:
            if isinstance(d[k], dict):
                self.__dict__[k] = DictDotLookup(d[k])
            elif isinstance(d[k], (list, tuple)):
                l = []
                for v in d[k]:
                    if isinstance(v, dict):
                        l.append(DictDotLookup(v))
                    else:
                        l.append(v)
                self.__dict__[k] = l
            else:
                self.__dict__[k] = d[k]

    def __getitem__(self, name):
        if name in self.__dict__:
            return self.__dict__[name]

    def __iter__(self):
        return iter(self.__dict__.keys())

    def __repr__(self):
        return pprint.pformat(self.__dict__)

def dict2obj(dict_in):
    obj = myStruct()
    for key in dict_in.keys():
        print key, ":", dict_in[key]
        if type(dict_in[key]) is dict:
            setattr(obj, key, dict2obj(dict_in[key]))
            #obj.key = dict2obj(dict_in[key])
        else:
            setattr(obj, key, dict_in[key])
            #obj.key = dict_in[key]
    return obj

class myStruct(object):
    pass

###########
def j2p(x):
    """j2p creates a pythonic interface to nested arrays and
    dictionaries, as returned by json readers.

    >>> a = { 'x':[5,8], 'y':5}
    >>> aa = j2p(a)
    >>> aa.y=7
    >>> print a
    {'x': [5, 8], 'y':7}
    >>> aa.x[1]=99
    >>> print a
    {'x': [5, 99], 'y':7}

    >>> aa.x[0] = {'g':5, 'h':9}
    >>> print a
    {'x': [ {'g':5, 'h':9} , 99], 'y':7}
    >>> print aa.x[0].g
    5
    """
    if isinstance(x, list):
        return _list_proxy(x)
    elif isinstance(x, dict):
        return _dict_proxy(x)
    else:
        return x

class _list_proxy(object):
    def __init__(self, proxied_list):
        object.__setattr__(self, 'data', proxied_list)
    def __getitem__(self, a):
        return j2p(object.__getattribute__(self, 'data').__getitem__(a))
    def __setitem__(self, a, v):
        return object.__getattribute__(self, 'data').__setitem__(a, v)


class _dict_proxy(_list_proxy):
    def __init__(self, proxied_dict):
        _list_proxy.__init__(self, proxied_dict)
    def __getattribute__(self, a):
        return j2p(object.__getattribute__(self, 'data').__getitem__(a))
    def __setattr__(self, a, v):
        return object.__getattribute__(self, 'data').__setitem__(a, v)


def p2j(x):
    """p2j gives back the underlying json-ic json-ic nested
    dictionary/list structure of an object or attribute created with
    j2p.
    """
    if isinstance(x, (_list_proxy, _dict_proxy)):
        return object.__getattribute__(x, 'data')
    else:
        return x

#############
class Struct(object):
    def __init__(self, d):
        self.d = d
    def __getattr__(self, key):
        return self.d[key]

class dotdictify(dict):
    def __init__(self, value=None):
        if value is None:
            pass
        elif isinstance(value, dict):
            for key in value:
                self.__setitem__(key, value[key])
        else:
            raise TypeError, 'expected dict'

    def __setitem__(self, key, value):
        if '.' in key:
            myKey, restOfKey = key.split('.', 1)
            target = self.setdefault(myKey, dotdictify())
            if not isinstance(target, dotdictify):
                raise KeyError, 'cannot set "%s" in "%s" (%s)' % (restOfKey, myKey, repr(target))
            target[restOfKey] = value
        else:
            if isinstance(value, dict) and not isinstance(value, dotdictify):
                value = dotdictify(value)
            dict.__setitem__(self, key, value)

    def __getattr__(self,key):
        try:
            return self._response[key]
        except KeyError,err:
            raise AttributeError(key)

    def __getitem__(self, key):
        if '.' not in key:
            #print "dict  :", str(self)
            return self.__getattr__(key)
            #return dict.getattr(key)
        myKey, restOfKey = key.split('.', 1)
        target = dict.__getitem__(self, myKey)
        if not isinstance(target, dotdictify):
            raise KeyError, 'cannot get "%s" in "%s" (%s)' % (restOfKey, myKey, repr(target))
        return target[restOfKey]

    def __contains__(self, key):
        if '.' not in key:
            return dict.__contains__(self, key)
        myKey, restOfKey = key.split('.', 1)
        target = dict.__getitem__(self, myKey)
        if not isinstance(target, dotdictify):
            return False
        return restOfKey in target

    def setdefault(self, key, default):
        if key not in self:
            self[key] = default
        return self[key]

    __setattr__ = __setitem__
    __getattr__ = __getitem__

# instances of this class are created for every externally advertised service
class ROS_Service_Template( threading.Thread):
    service_request_timeout = 600           #seconds
    check_response_delay = 0.1              #seconds
    wait_for_busy_service_provider = 0.1    #seconds

    service_name = None
    service_node_name = None
    service_module = None
    service_type = None
    client_callback = None

    client_id = None
    service_id = None

    ros_serviceproxy = None

    request_counter = 0
    request_list = {}     # holds requests until they are answered (response successfully sent to ROS-client)
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
        
        self.spawn_ROS_service( service_module, service_type, service_name, client_id)


    def handle_service_request(self, req):

        print "req"
        print req

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
        self.request_counter = (self.request_counter + 1) % 500000


        #msg_instance = populate_instance(req, self.service_class)
        values = extract_values(req)

        print "values:"
        print values

        # TODO: check for more complex parameters and types --> better argument parser!
        # --> see message_conversion
#        args_list = str(req).split("\n")
#        args_dict = {}
#        for arg in args_list:
#            key, value = arg.split(":")
#            #args_dict[key.strip()] = value.strip()
#            args_dict[key] = value

        request_message_object = {"op":"service_request",
                                    "request_id": request_id,
                                    "args": values #args_dict
                                    }
        # add request to request_list
        if request_id not in self.request_list.keys():
            self.request_list[request_id] = request_message_object
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
                print "received JSON:"
                print answer
                # remove response from response_list
                del self.response_list[request_id]
            else:
                # request failed due to timeout (or some other reason?!)
                print "request timed out!"
                answer = None
            # remove request from request_list
            del self.request_list[request_id]
        # TODO: more detailed exception handling
        except Exception, e:
            print e
        self.busy = False

        #answer = Struct(**answer)
        #answer = Struct(answer)
        #answer = dict2obj(answer)
        #answer = DictDotLookup(answer)
        #answer = j2p(answer)

        #print "answer:", str(answer)
        #print "test", answer.snake_arm_config_trajectory.data

        return answer

    def stop_ROS_service(self):
        self.spawned = False
        self.finish_flag = True
        self.ros_serviceproxy.shutdown("reason: stop_service requested by providing client")
        # set answer for unfinished requests to None
        # --> response can be found, but will be the same as for failed/timed out requests..
        for request_id in self.request_list.keys():
            self.response_list[request_id] = None
        # wait for request_loops to run into finish_flags
        while len(self.request_list) > 0:
            time.sleep(self.check_response_delay)
        # remove service from service_list
        service_list = ServiceList().list
        del service_list[self.service_name]
        self.protocol.log("info", "removed service: "+ self.service_name)

    service_class = None

    def spawn_ROS_service(self, service_module, service_type, service_name, client_id):
        # import service type for ros-service that we want to register in ros
        self.service_class = get_service_class(service_module+'/'+service_type)
        # register service in ros
        self.ros_serviceproxy = rospy.Service( service_name, self.service_class, self.handle_service_request)

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

    def __init__(self, protocol):
        self.protocol = protocol
        Capability.__init__(self, protocol)
        protocol.register_operation(self.opcode_advertise_service, self.advertise_service)

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
