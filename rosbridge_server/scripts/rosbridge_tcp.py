#!/usr/bin/env python

from rospy import init_node, get_param, loginfo, logerr
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

from signal import signal, SIGINT, SIG_DFL
import socket, subprocess, re

def get_ipv4_address():
    """
    Returns IP address(es) of current machine.
    """
    p = subprocess.Popen(["ifconfig"], stdout=subprocess.PIPE)
    ifc_resp = p.communicate()
    patt = re.compile(r'inet\s*\w*\S*:\s*(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})')
    resp = patt.findall(ifc_resp[0])[0]
    #print resp
    return resp
import SocketServer
import sys

#TODO: take care of socket timeouts and make sure to close sockets after killing programm to release network ports

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0

# Maximum length per message for incoming data
max_msg_length = 20000
socket_timeout = 60

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

    def send_message(self, message):
        """
        Callback from rosbridge
        """
        # Send data to TCP socket
        self.request.send(message)

if __name__ == "__main__":
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
    my_ip = get_ipv4_address()
    print "server-ip:", my_ip
    server = SocketServer.ThreadingTCPServer((my_ip, port), RosbridgeTcpSocket)

    loginfo("Rosbridge TCP server started on port %d", port)

    server.serve_forever()
