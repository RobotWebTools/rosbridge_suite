#!/usr/bin/python
import socket
import time

from random import randint
try:                       # try to import json-lib: 1st try usjon, 2nd try simplejson, else import standard python json
    import ujson as json
except ImportError:
    try:
        import simplejson as json
    except ImportError:
        import json




# TODO: handle multiple service requests at the same time

####################### variables begin ########################################
# these parameters should be changed to match the actual environment           #
################################################################################

tcp_socket_timeout = 10                         # seconds
max_msg_length = 20000                           # bytes

rosbridge_ip = "localhost"                      # hostname or ip
rosbridge_port = 9090                           # port as integer

service_module = "rosbridge_library.srv"        # make sure srv and msg files are available within specified module on rosbridge-server!
service_type = "SendBytes"                     # make sure this matches an existing service type on rosbridge-server (in specified srv_module)
service_name = "send_bytes2"                   # service name

send_fragment_size = 10
send_fragment_delay = 0.1
receive_fragment_size = 10

####################### variables end ##########################################



####################### service_calculation begin ##############################
# change this function to match whatever service should be provided            #
################################################################################

def calculate_service_response(request):
    request_object = json.loads(request)                                # parse string for service request
    print "request_message:", request_object

    count = int(request_object["args"]["count"] )                                    # do service calculation


    print "count:", count
    message = ""
    for i in range(0,count):
        message += str(chr(randint(32,126)))
#        message += "}{"
        message = message.replace("}","-")
        message = message.replace("{","-")
        if i% 100000 == 0:
            print count - i, "bytes left to generate"

    #print "message:", message


    #time.sleep(5)                                                      # to test service_response_timeouts
    service_response_data = { "data": message }                              # service response (as defined in srv-file)

    response_object = { "op": "service_response",
                        "request_id": request_object["request_id"],
                        "data": service_response_data
                      }
    response_message = json.dumps(response_object)
    #print "response_message:",response_message
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
                                "service_name": service_name,
                                "fragment_size": receive_fragment_size
                                }
    advertise_message = json.dumps(advertise_message_object)                    
    tcp_socket.send(str(advertise_message))

def unadvertise_service():                                                      # unadvertise service
    unadvertise_message_object = {"op":"stop_service",                           
                                  "service_name": service_name
                                 }
    unadvertise_message = json.dumps(unadvertise_message_object)                   
    tcp_socket.send(str(unadvertise_message))

def findBrackets( aString ):
   #print "find brackets:"
   #print aString
   if '{' in aString:
      match = aString.split('{',1)[1]
      open = 1
      for index in xrange(len(match)):
         if match[index] in '{}':
            open = (open + 1) if match[index] == '{' else (open - 1)
         if not open:
            #print "find brackets returns:", match[:index]
            return "["+match[:index]+"]"

import sys

buffer = ""

def wait_for_service_request():                                                 # receive data from rosbridge
    data = None
    global buffer

    # TODO: only check for fragmentlists.. server side blocking of busy service provider!

    try:
        done = False
        global buffer
        #buffer = ""
        while not done:
            incoming = tcp_socket.recv(max_msg_length)
            if incoming == '':
                print "connection closed by peer"
                sys.exit(1)
            if buffer == "":
                buffer = incoming
            else:
                buffer = buffer + incoming
            #print "incoming:",incoming
            print "buffer:", buffer
            # try to access service_request directly (not fragmented)
            try:
                data_object = json.loads(buffer)
                if data_object["op"] == "service_request":
                    data = buffer
                    done = True
                    return data
            except Exception, e:
                print "direct_access error:"
                print e
                #if json.loads(buffer)["values"] == None:
                    #print "server replaced services"
                    #sys.exit(1)
                

            # opcode was not "service_request" -> try to defragment
            print "trying to defragment"
            try:
                
                #result = json.loads("["+buffer+"]")

                #result = nestedExpr('{','}').parseString(buffer).asList()
                #result_string = findBrackets("{"+buffer+"}")
                result_string = buffer.split("}{")
                #print "split by }{;",result_string
                result = []
                for fragment in result_string:
                    if fragment[0] != "{":
                        fragment = "{"+fragment
                    if fragment[len(fragment)-1] != "}":
                        fragment = fragment + "}"
                    result.append(json.loads(fragment))
                #result = json.loads(str(result_string))
                #print "result:", result
                try:
                    fragment_count = len(result)
                    announced = int(result[0]["total"])

                    if fragment_count == announced:
                        reconstructed = ""
                        #print "unsorted list of fragments:", result
                        # TODO: sort fragments before reconstructing!!

                        sorted_result = [None] * fragment_count
                        unsorted_result = []
                        for fragment in result:
                            unsorted_result.append(fragment)
                            sorted_result[int(fragment["num"])] = fragment
                        print "unsorted_list:", unsorted_result
                        print "sorted_list:", sorted_result

                        for fragment in sorted_result:
                            reconstructed = reconstructed + fragment["data"]

                        print "reconstructed", reconstructed
                        buffer = ""
                        done = True
                        print "reconstructed message from", len(result), "fragments"
                        #print reconstructed
                        return reconstructed
                except Exception, e:
                    print e
                    print "not possible to defragment:", buffer
                    # try to devide into multiple json objects
            except Exception, e:
                print "defrag_error:", buffer
                print e
                pass
    except Exception, e:
        #print "network-error(?):", e
        pass
    return data

def send_service_response(response):                                            # send response to rosbridge
    tcp_socket.send(response)




#  create fragment messages for a huge message #################################
def list_of_fragments(full_message, fragment_size):

    print "message length:", len(full_message)

    message_id = randint(0,64000)                                               # generate random message id

    fragments = []                                                              # generate list of data fragments
    cursor = 0
    while cursor < len(full_message):
        fragment_begin = cursor
        if len(full_message) < cursor + fragment_size:
            fragment_end = len(full_message)
            cursor = len(full_message)
        else:
            fragment_end = cursor + fragment_size
            cursor += fragment_size
        fragment = full_message[fragment_begin:fragment_end]
        fragments.append(fragment)

    #print "fragment_list:", fragments

    fragmented_messages_list = []                                               # generate list of fragmented messages

    if len(fragments) > 1:

        for count, fragment in enumerate(fragments):                                # iterate through list and have index counter
            fragmented_message_object = {"op":"fragment",                           #   create python-object for each fragment message
                                         "id": str(message_id),
                                         "data": str(fragment),
                                         "num": count,
                                         "total": len(fragments)
                                         }

            fragmented_message = json.dumps(fragmented_message_object)              # create JSON-object from python-object for each fragment message
            fragmented_messages_list.append(fragmented_message)                     # append JSON-object to list of fragmented messages
    else:
        fragmented_messages_list.append(str(fragment))
        
    #print "fragment_messages_list:", fragmented_messages_list
    return fragmented_messages_list                                             # return list of 'ready-to-send' fragmented messages
## fragmentation example end ###################################################

####################### helper functions end ###################################



####################### script begin ###########################################
# should not need to be changed (but could be improved )                            #
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

            print "response calculated, now splitting into fragments.."
            fragment_list = list_of_fragments(response, send_fragment_size)


            print "sending", len(fragment_list), "messages as response"
            for fragment in fragment_list:
                print "sending:" ,fragment
                send_service_response(fragment)                 # send service_response to rosbridge
                time.sleep(send_fragment_delay)
        except Exception, e: # allows to try to receive next request/data
          print e
          pass
except KeyboardInterrupt:
    try:
        unadvertise_service()                               # unadvertise service
        tcp_socket.close()                                                      # close tcp_socket
    except Exception, e:
        print e
    print "non-ros_service_server stopped because user pressed \"Ctrl-C\""
