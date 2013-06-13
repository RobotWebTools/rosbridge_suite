#!/usr/bin/python
import socket
import time

# ROS imports
#import roslib
#roslib.load_manifest('beginner_tutorials')
#import rospy

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
#host1_sock.send('raw\r\n\r\n')

#tell the socket what topics should be subscribed (i.e. camera voltage)
#host1_sock.send('\x00{"receiver":"/rosbridge/subscribe","msg":["/#flexible_joint_states",-1,"sensor_msgs/JointState"]}\xff')

host1_sock.send('{"op": "subscribe", "topic": "nonrosfragmentedchatter", "type": "std_msgs/String"}')

while True:
    incoming = ""
    try:
        incoming = host1_sock.recv(1024000)#.decode('unicode-escape')
        #incoming = incoming[1:len(incoming)-1]
        #incoming contains the messages on the subscribed topics
        
        #msg_object = json.loads(incoming)
        #print "data: " + msg_object["msg"]["data"]

        print "+++++++++++++++++++++"
        print json.loads(incoming)["msg"]["data"]
        print "+++++++++++++++++++++"
        print "received message length:",  len(json.loads(incoming)["msg"]["data"])
        print "+++++++++++++++++++++"
    except Exception, e:
        print "---------------------"
        print incoming
        print "---------------------"
        print "received message length:",  len(incoming)
        print "Exception occured:"
        print e
        print "---------------------"

#    #publish something (i.e. data from sps like odometry)
#    host1_sock.send('\x00{"receiver":"/bridge_topic", "msg":{"data": 42}, "type":"std_msgs/#Int32"}\xff')        
#    host1_sock.send("""{op: 'publish', topic:'/chatter', msg:'rosbridge over tcp-socket'}""")
    time.sleep(0.05)
