#!/usr/bin/python
import socket

try:
    import ujson as json
except ImportError:
    try:
        import simplejson as json
    except ImportError:
        import json


####################### variables begin ########################################
# these parameters should be changed to match the actual environment           #
################################################################################

client_socket_timeout = 60                      # seconds
max_msg_length = 1024000                        # bytes

rosbridge_ip = "localhost"                      # hostname or ip
rosbridge_port = 9090                           # port as integer

service_module = "rosbridge_library.srv"        # make sure srv and msg files are available within specified module on rosbridge-server!
service_type = "AddTwoInts"                     # make sure this matches an existing service type on rosbridge-server (in specified srv_module)
service_name = "add_two_ints"                   # service name

####################### variables end ##########################################


################################################################################

def request_service():
    service_request_object = { "op" : "call_service",
                               "service": "/add_two_ints",
                               "args": { "a" : 3,
                                         "b" : 5
                                        }
                              }
    service_request = json.dumps(service_request_object)
    print "sending JSON-message to rosbridge:", service_request
    sock.send(service_request)

################################################################################


####################### script begin ###########################################
# should not need to be changed (but could be improved ;) )                    #
################################################################################

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                        #connect to rosbridge
sock.settimeout(client_socket_timeout)
sock.connect((rosbridge_ip, rosbridge_port))

request_service()                                                               # send service_request

incoming = None
while incoming == None:     # should not need a loop (maximum wait can be set by client_socket_timeout), but since its for test/demonstration only .. leave it as it is for now
    try:
        incoming = sock.recv(max_msg_length)                                    # receive service_response from rosbridge
        print "+++++++++++++++++++++"
        service_response = json.loads(incoming)                                 # service_response contains JSON service response as sent by rosbridge
        print "op:", service_response["op"]
        print "service_name:", service_response["service"]
        print "values:", service_response["values"]
        print "+++++++++++++++++++++"
        break
    except Exception, e:
        print "---------------------"
        print incoming
        print "---------------------"
        print "received message length:",  len(incoming)
        print "Exception occured:"
        print e
        print "---------------------"

sock.close()                                                                    # close socket

