#!/usr/bin/python
import socket
import time

# this script uses the already existing "call_service"-functionality of rosbridge_suite
# this is only suitable for testing "non-ros service client to non-ros service server calls"
#  if the targeted service server is started via rosbridge (see script: non-ros_service_server.py")

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

client_socket_timeout = 60

#connect to the socket
host1_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host1_sock.settimeout(client_socket_timeout)
host1_sock.connect(('localhost', 9090))

service_request_object = { "op" : "call_service",
						   "service": "/add_two_ints",
						   "args": { "a" : 3,
								 	 "b" : 5
								   }
						 }
						 
service_request = json.dumps(service_request_object)
print "sending JSON-message to rosbridge:", service_request

host1_sock.send(service_request)

incoming = None
# should not need a loop, but since its for test/demonstration only .. leave it as it is for now
while incoming == None:
	# probably obsolete
    incoming = None
    try:
        incoming = host1_sock.recv(1024000)
        print "+++++++++++++++++++++"
        print "op:", json.loads(incoming)["op"]
        print "service_name:", json.loads(incoming)["service"]
        print "values:", json.loads(incoming)["values"]
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

    time.sleep(5)
