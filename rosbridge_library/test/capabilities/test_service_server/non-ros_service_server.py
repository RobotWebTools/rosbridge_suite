#!/usr/bin/python

import socket
import time
from datetime import datetime
import sys

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




delay_between_socket_sends = 5           # seconds !rosbridge messes up incoming messages if this delay is too short
max_msg_length = 1024
rosbridge_ip = "localhost"
rosbridge_port = 9090

host1_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                  # connect to rosbridge
host1_sock.settimeout(10)
host1_sock.connect((rosbridge_ip, rosbridge_port))

advertise_message_object = {"op":"advertise_service",                           # 
                            "service_module": "rosbridge_library.srv",
                            "service_type": "AddTwoInts",
                            "service_name": "add_two_ints"
                            }
advertise_message = json.dumps(advertise_message_object)                        #   

host1_sock.send(str(advertise_message))                                         # 
time.sleep(1)

#    self.request.settimeout(socket_timeout)

try:
    while True:                                                                 # loop forever (or until ctrl-c is pressed)
        data = ""
        try:
          data = host1_sock.recv(max_msg_length)
          if data.strip() == '':                                                # exit on empty string
              # TODO: handle case that rosbridge cancels connection
              break
          elif len(data.strip()) > 0:
            begin = datetime.now()                                              # timestamp for begin of processing
            request_object = json.loads(data)                                   # parse string for service request
            print "request received:", request_object

            a = request_object["args"]["a"]                                     # do service calculation
            b = request_object["args"]["b"]
            sum = int(a) + int(b)

            time.sleep(5)
            service_response_data = { "sum": sum }                              # service response (as defined in srv-file)
            response_object = { "op": "service_response",
                                "request_id": request_object["request_id"],
                                "data": service_response_data
                              }
            response_message = json.dumps(response_object)
            print "response_message:",response_message

            host2_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)      # send service response via a new socket to rosbridge
            host2_sock.settimeout(10)
            host2_sock.connect((rosbridge_ip, rosbridge_port))
            host2_sock.send(response_message)                                   
            host2_sock.close()

            # to check serverside processing time
            duration = (datetime.now() - begin).total_seconds()
            #print "duration: ", duration
          else:
              pass
        except Exception, e:
          #print "socket connection timed out! (ignore warning if client is only listening..)"
          pass

        #time.sleep(delay_between_socket_sends)                          

        # wait for a while before creating next huge message
        #time.sleep(delay_between_big_messages)
except KeyboardInterrupt:
    try:
        unadvertise_message_object = {"op":"stop_service",                                   # advertise topic
                                      "service_name": "add_two_ints"
                                     }
        unadvertise_message = json.dumps(unadvertise_message_object)                        #   create JSON-object (which is the advertise message)

        host1_sock.send(str(unadvertise_message))
    except Exception, e:
        print e

    print "non-ros_service_server aborted"
