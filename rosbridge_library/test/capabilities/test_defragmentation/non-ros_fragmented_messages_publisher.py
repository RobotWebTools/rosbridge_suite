#!/usr/bin/python

# This script needs tcp2ws wrapper running!!
#   [tcp2ws is a tcp-wrapper to allow non-ros clients to connect to rosbridge via
#    tcp-socket instead of using web-sockets]

# example script for fragmented messages:
# - op:         fragment
# - id:         generate message ID to identify which fragments belong to a message
# - data:       split full message into parts (just split the string containing the whole original message..)
# - num:        generate 'fragment ID' for each fragment (in order to reconstruct the full message)
# - total:      defines how many fragments belong to a fragmented message


import socket
import time
from random import randint

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

# options
message_length = 52428 #80                    # bytes / letters / ..

#print "parameters given: " + str(len(sys.argv)-1)

if len(sys.argv) == 2:
    try:
        message_length = int(sys.argv[1])   # coming as string from shell..
    except Exception, e:
        pass

print "using message length: " +str(message_length)

fragment_length = 1000                      # bytes / letters / ..
tcp2ws_port = 9090                          # <- int
tcp2ws_ip = "localhost"                     # <- string
delay_between_big_messages = 5              # seconds
delay_between_socket_sends = 0.05           # seconds !rosbridge messes up incoming messages if this delay is too short


#  create fragment messages for a huge message #################################
def list_of_fragments():
    full_message_data = ""                                                      # generate full_message_data
    for i in range(0,message_length):
        full_message_data += str(chr(randint(32,126)))                              # append random capital letter to string
    # debug
    full_message_data += "\nsent@: " + datetime.now().strftime("%H:%M:%S")

    full_message_data = full_message_data#.encode('unicode-escape')

    print "message length:", len(full_message_data)


#    msg_begin =  """{"op":"publish","""                                         # generate full message with full data and headers
#    msg_begin += """"topic":"nonrosfragmentedchatter","msg":{"data":"""
#    msg_end   =  """}}"""
#    full_message = msg_begin + "\"" + full_message_data + "\"" +  msg_end

    full_message_obj = {"op":"publish",
                        "topic":"nonrosfragmentedchatter",
                        "msg":{"data": str(full_message_data)
                               }
                        }
    full_message = json.dumps(full_message_obj)

    print "json-test message length:",  len(json.loads(full_message)["msg"]["data"])

    message_id = randint(0,64000)                                               # generate random message id

    fragments = []                                                              # generate list of data fragments
    cursor = 0
    while cursor < len(full_message):
        fragment_begin = cursor
        if len(full_message) < cursor + fragment_length:
            fragment_end = len(full_message)
            cursor = len(full_message)
        else:
            fragment_end = cursor + fragment_length
            cursor += fragment_length
        fragment = full_message[fragment_begin:fragment_end]
        fragments.append(fragment)

    fragmented_messages_list = []                                               # generate list of fragmented messages
    for count, fragment in enumerate(fragments):                                # iterate through list and have index counter
        fragmented_message_object = {"op":"fragment",                           #   create python-object for each fragment message
                                     "id": str(message_id),
                                     "data": str(fragment),
                                     "num": count,
                                     "total": len(fragments)
                                     }

        fragmented_message = json.dumps(fragmented_message_object)              # create JSON-object from python-object for each fragment message
        fragmented_messages_list.append(fragmented_message)                     # append JSON-object to list of fragmented messages

    return fragmented_messages_list                                             # return list of 'ready-to-send' fragmented messages
## fragmentation example end ###################################################

host1_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                  # connect to the socket (tcp2ws)
host1_sock.settimeout(1)
host1_sock.connect((tcp2ws_ip, tcp2ws_port))

advertise_message_object = {"op":"advertise",                                   # advertise topic
                            "topic":"nonrosfragmentedchatter",                  #   create python-object
                            "type":"std_msgs/String"
                            }
advertise_message = json.dumps(advertise_message_object)                        #   create JSON-object (which is the advertise message)

host1_sock.send(str(advertise_message))                                         # send advertise message
time.sleep(1)

try:
    while True:                                                                 # loop forever (or until ctrl-c is pressed)
        fragments = 0
        begin = datetime.now()
        try:
            fragment_list = list_of_fragments()
            fragment_count = len(fragment_list)
            last_percent = 0
            for message in fragment_list:                                 # create new list of fragments and iterate over each fragment
                fragments += 1

                host1_sock.send(str(message))                                   # send fragment message

                # if 10, 20, 30 .. 100% reached..
                percent = (fragments * 100 / fragment_count)
                if percent %10 == 0 and percent > last_percent:
                    last_percent = percent
                    print str(percent) +"% sent. ["+str(fragments) + " of " + str(fragment_count) + " fragments]"

                time.sleep(delay_between_socket_sends)                          # wait for a moment, so that network is not 'flooded' too much
                
        except Exception, e:
            print e
            print "an error occured"

        duration = (datetime.now() - begin).total_seconds()

        print fragments," message fragments sent. [duration: " + str(duration) +"s]"
        fragments = 0
        # wait for a while before creating next huge message
        #time.sleep(delay_between_big_messages)
except KeyboardInterrupt:
    print "non-ros-publisher aborted"
