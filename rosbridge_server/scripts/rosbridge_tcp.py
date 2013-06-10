#!/usr/bin/env python

from rospy import init_node, get_param, loginfo, logerr
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

from signal import signal, SIGINT, SIG_DFL
import SocketServer
import sys

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0

# Maximum length per message for incoming data
max_msg_length = 2048

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
        while 1:
            data = self.request.recv(max_msg_length)
            # Exit on empty string
            if data.strip() == '':
                break
            else:
                self.protocol.incoming(data)

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
    init_node("rosbridge_server")
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
    server = SocketServer.ThreadingTCPServer(('localhost', port), RosbridgeTcpSocket)

    loginfo("Rosbridge server started on port %d", port)

    server.serve_forever()