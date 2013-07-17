#!/usr/bin/env python

from rospy import init_node, get_param, loginfo, logerr
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

from signal import signal, SIGINT, SIG_DFL
import socket, subprocess, re

import SocketServer
import sys

#TODO: take care of socket timeouts and make sure to close sockets after killing programm to release network ports

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0

# Maximum length per socket_read for incoming data
max_msg_length = 20000000
socket_timeout = 10

send_delay = 0.1

class RosbridgeTcpSocket(SocketServer.BaseRequestHandler):
    """
    TCP Socket server for rosbridge
    """
    def setup(self):
        global client_id_seed, clients_connected
        try:
            self.protocol = RosbridgeProtocol(client_id_seed)
            self.protocol.outgoing = self.send_message
            client_id_seed = client_id_seed + 1
            clients_connected = clients_connected + 1
        except Exception as exc:
            logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        loginfo("Client connected.  %d clients total.", clients_connected)

    def handle(self):
        """
        Listen for TCP messages
        """
        self.request.settimeout(socket_timeout)
        while 1:
            try:
              data = self.request.recv(max_msg_length)
              # Exit on empty string
              if data.strip() == '':
                  break
              elif len(data.strip()) > 0:
                  #time.sleep(5)
                  self.protocol.incoming(data.strip(''))
              else:
                  pass
            except Exception, e:
                pass
                self.protocol.log("debug", "socket connection timed out! (ignore warning if client is only listening..)")

    def finish(self):
        """
        Called when TCP connection finishes
        """
        global clients_connected
        clients_connected = clients_connected - 1
        self.protocol.finish()
        loginfo("Client disconnected.  %d clients total.", clients_connected)        

    busy = False
    queue = []
    # TODO: cleaner
    def send_message(self, message=None):
        """
        Callback from rosbridge
        """

        if self.busy:
            if message!=None:
                print "adding to queue"
                self.queue.append(message)
        else:

            self.busy = True
            

            if message == None and len(self.queue) > 0:
                message = self.queue[0]
                self.queue = self.queue[1:]
                #print self.queue
            if message != None:
                # Send data to TCP socket
                #print "waiting; queue-len:",len(self.queue), self.protocol.client_id
                #print "sending:",message
                self.request.send(message)
                time.sleep(send_delay)
                
            self.busy = False
            if len(self.queue) > 0:
                print "accessing queue", len(self.queue)
                self.send_message()


#        self.busy = False

import time

if __name__ == "__main__":
    loaded = False
    retry_count = 0
    #my_ip = get_ipv4_address()
    #print "server-ip:", my_ip
    while not loaded:
        retry_count += 1
        print retry_count
        try:
            init_node("rosbridge_tcp")
            signal(SIGINT, SIG_DFL)

            port = get_param('/rosbridge/port', 9090)
            if "--port" in sys.argv:
                idx = sys.argv.index("--port") + 1
                if idx < len(sys.argv):
                    port = int(sys.argv[idx])
                else:
                    print "--port argument provided without a value."
                    sys.exit(-1)

            # Server host is a tuple ('host', port)
            # empty string for host makes server listen on all available interfaces
            server = SocketServer.ThreadingTCPServer(("", port), RosbridgeTcpSocket)

            loginfo("Rosbridge TCP server started on port %d", port)

            server.serve_forever()
            loaded = True
        except Exception, e:
            #print "remove me", e
            #print "waiting 1 second before retrying.."
            time.sleep(1)
    print "server loaded"
