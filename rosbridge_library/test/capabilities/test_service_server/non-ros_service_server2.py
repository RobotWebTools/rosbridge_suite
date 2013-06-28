#!/usr/bin/python




import socket
import time

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

from datetime import datetime
import sys


delay_between_socket_sends = 5           # seconds !rosbridge messes up incoming messages if this delay is too short


host1_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                  # connect to the socket (tcp2ws)
host1_sock.settimeout(30)
host1_sock.connect(("localhost", 9090))

advertise_message_object = {"op":"advertise_service",                                   # advertise topic
#                            "service_node_name":"nonrosserviceserver",                  #   create python-object
                            "service_module": "beginner_tutorials.srv",
                            "service_type": "AddTwoInts",
                            "service_name": "add_two_ints2"
                            }
advertise_message = json.dumps(advertise_message_object)                        #   create JSON-object (which is the advertise message)

host1_sock.send(str(advertise_message))                                         # send advertise message
time.sleep(1)

try:
#    self.request.settimeout(socket_timeout)
    while True: # loop forever (or until ctrl-c is pressed)
        data = ""
        try:
          data = host1_sock.recv(max_msg_length)
          # Exit on empty string
          if data.strip() == '':
              break
          elif len(data.strip()) > 0:
            # parse string for service request


            # timestamp for begin of processing
            begin = datetime.now()            
            
            # do service calculation
            
            
            # create answer
            message = data


            # send answer
            host1_sock.send(str(message))                                   # send fragment message


            # to check serverside processing time
            duration = (datetime.now() - begin).total_seconds()
            print "duration: ", duration
          else:
              pass
        except Exception, e:
          print "socket connection timed out! (ignore warning if client is only listening..)"



        time.sleep(delay_between_socket_sends)                          # wait for a moment, so that network is not 'flooded' too much

        

        
        
        # wait for a while before creating next huge message
        #time.sleep(delay_between_big_messages)
except KeyboardInterrupt:
    print "non-ros_service_server aborted"
