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

client_socket_timeout = 6                      # seconds
max_msg_length = 2000000                        # bytes

rosbridge_ip = "localhost"                       # hostname or ip
rosbridge_port = 9095                           # port as integer

service_name = "send_bytes"                   # service name
request_byte_count = 50000
receiving_fragment_size = 100

####################### variables end ##########################################


################################################################################



def request_service():
    service_request_object = { "op" : "call_service",
                               "service": "/"+service_name,
                               "fragment_size": receiving_fragment_size,
                               "args": { "count" : request_byte_count
                                        }
                              }
    service_request = json.dumps(service_request_object)
    print "sending JSON-message to rosbridge:", service_request
    sock.send(service_request)

################################################################################


####################### script begin ###########################################
# should not need to be changed (but could be improved ;) )                    #
################################################################################
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                        #connect to rosbridge
    sock.settimeout(client_socket_timeout)
    sock.connect((rosbridge_ip, rosbridge_port))

    request_service()                                                               # send service_request

    incoming = None
    buffer = ""
    done = False
    result = None
    reconstructed = None
    while not done:     # should not need a loop (maximum wait can be set by client_socket_timeout), but since its for test/demonstration only .. leave it as it is for now
        try:
            incoming = sock.recv(max_msg_length)                                    # receive service_response from rosbridge
            if buffer == "":
                buffer = incoming
                if incoming == "":
                    print "closing socket"
                    sock.close()
                    break
            else:
                buffer = buffer + incoming

            print "buffer-length:", len(buffer)

            # try to access service_request directly (not fragmented)
            try:
                data_object = json.loads(buffer)
                if data_object["op"] == "service_response":
                    reconstructed = buffer
                    done = True

            except Exception, e:
                #print "direct access to JSON failed.."
                #print e
                pass


    # TODO: if opcode is fragment --> defragment, else access service request directly

    # TODO: use the same processing scheme for multiple/partial JSON as in protocol

            try:
                #print "defragmenting incoming messages"
                # TODO: allow "}{" in strings!
                result_string = buffer.split("}{")
                result = []
                for fragment in result_string:
                    if fragment[0] != "{":
                        fragment = "{"+fragment
                    if fragment[len(fragment)-1] != "}":
                        fragment = fragment + "}"
                        
                    try:
                        result.append(json.loads(fragment))
                    except Exception, e:
                        print e
                        print result_string
                        raise


                fragment_count = len(result)
                print "fragment_count:", fragment_count
                announced = int(result[0]["total"])
                if fragment_count == announced:
                    # sort fragments
                    sorted_result = [None] * fragment_count
                    unsorted_result = []
                    for fragment in result:
                        unsorted_result.append(fragment)
                        sorted_result[int(fragment["num"])] = fragment

                    reconstructed = ''
                    for fragment in sorted_result:
                        reconstructed = reconstructed + fragment["data"]

                    done = True
            except Exception, e:
                #print e
                pass

        except Exception, e:
#            print e
            pass


    returned_data = json.loads(reconstructed)
    if returned_data["values"] == None:
        print "response was None -> service was not available"
    else:
        print "received:"
        print reconstructed
    

except Exception, e:
    print "ERROR - could not receive service_response"

sock.close()                                                                    # close socket
