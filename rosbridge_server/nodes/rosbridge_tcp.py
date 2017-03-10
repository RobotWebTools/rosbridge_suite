#!/usr/bin/env python
from roslib import load_manifest
load_manifest('rosbridge_server')
from rospy import init_node, get_param, loginfo, logerr
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

from signal import signal, SIGINT, SIG_DFL
import SocketServer
import sys

#TODO:  keyfile & certfile not implemented! if SocketServer cannot handle this, maybe switch back to tornado with tcp-sockets
#TODO:  maybe increase/decrease max_msg_length to better tested value

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0

max_msg_length = 2048                                                           # sets maximum length for incoming data (per message)
port = 9090                                                                     # sets listening port (gets overwritten from parameter server or commandline)

class RosbridgeTcpSocket(SocketServer.BaseRequestHandler):
    # -> websocket.open
    def setup(self):                                                            # called whenever a connection is opened
        global client_id_seed, clients_connected
        try:
            self.protocol = RosbridgeProtocol(client_id_seed)
            self.protocol.outgoing = self.send_message                          # "register" self.send_message in self.protocol.outgoing
            client_id_seed = client_id_seed + 1                                 # set client ID
            clients_connected = clients_connected + 1                           # increment client counter
        except Exception as exc:
            logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        loginfo("Client connected.  %d clients total.", clients_connected)

    # -> websocket.on_message
    def handle(self):                                                           # "IOLoop" for SocketServer
        while 1:
            data = self.request.recv(max_msg_length)                            # receive data
            if data.strip() == '':                                              # if data is empty
                break                                                           #   -> stop this IOLoop
            else:                                                               # else
                self.protocol.incoming(data)                                    #   -> pass data to protocol.incoming()

    # -> websocket.on_close
    def finish(self):                                                           # called whenever a connection is closed
        global clients_connected
        clients_connected = clients_connected - 1                               # decrement client counter
        self.protocol.finish()                                                  # call protocol.finish()
        loginfo("Client disconnected.  %d clients total.", clients_connected)        

    # -> websocket.send_message
    def send_message(self, message):                                            # "callback" for rosbridge
        self.request.send(message)                                              # send data to tcp socket


if __name__ == "__main__":
    init_node("rosbridge_server")
    signal(SIGINT, SIG_DFL)

    port = get_param('/rosbridge/port', 9090)                                   # get port from ros-parameter-server
    if "--port" in sys.argv:                                                    # if port exists as commandline argument
        idx = sys.argv.index("--port")+1                                        #   -> check if valid value and use this as port
        if idx < len(sys.argv):
            port = int(sys.argv[idx])
        else:
            print "--port argument provided without a value."
            sys.exit(-1)

    #server host is a tuple ('host', port)
    server = SocketServer.ThreadingTCPServer(('localhost', port), RosbridgeTcpSocket)   # create SocketServer instance

    loginfo("Rosbridge server started on port %d", port)

    server.serve_forever()