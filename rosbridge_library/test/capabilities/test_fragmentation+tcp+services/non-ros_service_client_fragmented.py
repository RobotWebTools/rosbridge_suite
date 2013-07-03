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

service_name = "send_bytes"                   # service name

####################### variables end ##########################################


################################################################################

def request_service():
    service_request_object = { "op" : "call_service",
                               "service": "/"+service_name,
                               "fragment_size": 1024,
                               "args": { "count" : 400000
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
buffer = ""
done = False
result = None
while not done:     # should not need a loop (maximum wait can be set by client_socket_timeout), but since its for test/demonstration only .. leave it as it is for now
    try:
        incoming = sock.recv(max_msg_length)                                    # receive service_response from rosbridge
        if buffer == "":
            buffer = incoming
        else:
            buffer = buffer + "," + incoming
        print "incoming:",incoming
        #print "+++++++++++++++++++++"
        #service_response = json.loads(incoming)                                 # service_response contains JSON service response as sent by rosbridge
        #print "response:", service_response
        #print "+++++++++++++++++++++"
        
        try:
            result = json.loads("["+buffer+"]")
            fragment_count = len(result)
            announced = int(result[0]["total"])
            if fragment_count == announced:
                done = True
        except Exception, e:
            print "===="
            print "["+buffer+"]"
            print "###"
            print e
            print "###"
        # don't break after first receive if using fragment_size!
        #break
    except Exception, e:


        print "---------------------"
        print buffer
        print "---------------------"
        print "received message length:",  len(incoming)
        print "Exception occured:"
        print e
        print "---------------------"

print "result:", result

# TODO: sort before reconstructing!!!
reconstructed = ''
for fragment in result:
    print "  ", fragment["data"]
    reconstructed = reconstructed + fragment["data"]
print
print "reconstructed message :",reconstructed

## TODO: check why json arrives as string!
#reconstructed = reconstructed.strip('"')
#print "reconstructed message2:",reconstructed

returned_data = json.loads(reconstructed)
print "returned json:", returned_data

print
print "received:"
print "------------------------------------------------------"
for key, value in returned_data.iteritems():
    print
    print key, ":"
    print
    print value

#answer = returned_data["values"]

#print "service_answer:", json.dumps(answer)


sock.close()                                                                    # close socket

