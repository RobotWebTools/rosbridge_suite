#!/usr/bin/env python

from rospy import init_node, get_param, loginfo, logerr
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

from signal import signal, SIGINT, SIG_DFL

import SocketServer
import sys
import time

#TODO: take care of socket timeouts and make sure to close sockets after killing programm to release network ports

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0

# Maximum length per socket_read for incoming data
max_msg_length = 1024
socket_timeout = 10     #seconds

retry_startup_delay = 5 #seconds

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
            self.protocol.log("info", "connected. " + str(clients_connected) + " client total.")
        except Exception as exc:
            logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        

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
        self.protocol.log("info", "disconnected. " + str(clients_connected) + " client total." )

    def send_message(self, message=None):
        """
        Callback from rosbridge
        """
        self.request.send(message)


if __name__ == "__main__":
    loaded = False
    retry_count = 0
    while not loaded:
        retry_count += 1
        print "trying to start rosbridge TCP server.."
        try:
            print ""
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
            time.sleep(retry_startup_delay)
    print "server loaded"
