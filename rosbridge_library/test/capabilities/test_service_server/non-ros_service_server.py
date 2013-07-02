#!/usr/bin/python
import socket
try:                       # try to import json-lib: 1st try usjon, 2nd try simplejson, else import standard python json
    import ujson as json
except ImportError:
    try:
        import simplejson as json
    except ImportError:
        import json


####################### variables begin ########################################
# these parameters should be changed to match the actual environment           #
################################################################################

tcp_socket_timeout = 10                         # seconds
max_msg_length = 1024                           # bytes

rosbridge_ip = "localhost"                      # hostname or ip
rosbridge_port = 9090                           # port as integer

service_module = "rosbridge_library.srv"        # make sure srv and msg files are available within specified module on rosbridge-server!
service_type = "AddTwoInts"                     # make sure this matches an existing service type on rosbridge-server (in specified srv_module)
service_name = "add_two_ints"                   # service name

####################### variables end ##########################################


####################### service_calculation begin ##############################
# change this function to match whatever service should be provided            #
################################################################################

def calculate_service_response(request):
    request_object = json.loads(request)                                # parse string for service request
    print "request_message:", request_object

    a = request_object["args"]["a"]                                     # do service calculation
    b = request_object["args"]["b"]
    sum = int(a) + int(b)

    #time.sleep(5)                                                      # to test service_response_timeouts
    service_response_data = { "sum": sum }                              # service response (as defined in srv-file)
    response_object = { "op": "service_response",
                        "request_id": request_object["request_id"],
                        "data": service_response_data
                      }
    response_message = json.dumps(response_object)
    print "response_message:",response_message
    print
    return response_message

####################### service_calculation end ################################


####################### helper functions begin #################################
# should not need to be changed (but could be improved ;) )                    #
################################################################################

def connect_tcp_socket():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                # connect to rosbridge
    tcp_sock.settimeout(10)
    tcp_sock.connect((rosbridge_ip, rosbridge_port))
    return tcp_sock

def advertise_service():                                                        # advertise service
    advertise_message_object = {"op":"advertise_service",                       
                                "service_module": service_module,
                                "service_type": service_type,
                                "service_name": service_name
                                }
    advertise_message = json.dumps(advertise_message_object)                    
    tcp_socket.send(str(advertise_message))

def unadvertise_service():                                                      # unadvertise service
    unadvertise_message_object = {"op":"stop_service",                           
                                  "service_name": service_name
                                 }
    unadvertise_message = json.dumps(unadvertise_message_object)                   
    tcp_socket.send(str(unadvertise_message))

def wait_for_service_request():                                                 # receive data from rosbridge
    data = None
    try:
        data = tcp_socket.recv(max_msg_length)
    except Exception, e:
        #print "network-error(?):", e
        pass
    return data

def send_service_response(response):                                            # send response to rosbridge
    tcp_socket.send(response)

####################### helper functions end ###################################



####################### script begin ###########################################
# should not be changed (but could be improved ;) )                            #
################################################################################

tcp_socket = connect_tcp_socket()                                               # open tcp_socket
advertise_service()                                         # advertise service in ROS (via rosbridge)
print "service provider started and waiting for requests"

try:                                                                            # allows to catch KeyboardInterrupt
    while True:                                                                 # loop forever (or until ctrl-c is pressed)
        data = None
        try:                                                                    # allows to catch any Exception (network, json, ..)
          data = wait_for_service_request()                 # receive request from rosbridge
          if data == '':                                                        # exit on empty string
              break                                                             # TODO: handle case that rosbridge cancels connection
          elif data != None and len(data) > 0:                                  # received service_request (at least some data..)
            response = calculate_service_response(data)     # do service_calculation
            send_service_response(response)                 # send service_response to rosbridge
        except Exception, e:                                                    # allows to try to receive next request/data
          pass
except KeyboardInterrupt:
    try:
        unadvertise_service()                               # unadvertise service
        tcp_socket.close()                                                      # close tcp_socket
    except Exception, e:
        print e
    print "non-ros_service_server stopped because user pressed \"Ctrl-C\""
