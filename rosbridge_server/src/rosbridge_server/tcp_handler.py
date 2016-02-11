import rospy
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

import SocketServer

class RosbridgeTcpSocket(SocketServer.BaseRequestHandler):
    """
    TCP Socket server for rosbridge
    """

    busy = False
    queue = []
    client_id_seed = 0
    clients_connected = 0

    # list of parameters
    incoming_buffer = 65536                 # bytes
    socket_timeout = 10                     # seconds
    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes

    def setup(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size
        }

        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, parameters=parameters)
            self.protocol.outgoing = self.send_message
            cls.client_id_seed += 1
            cls.clients_connected += 1
            self.protocol.log("info", "connected. " + str(cls.clients_connected) + " client total.")
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))

    def handle(self):
        """
        Listen for TCP messages
        """
        cls = self.__class__
        self.request.settimeout(cls.socket_timeout)
        while 1:
            try:
              data = self.request.recv(cls.incoming_buffer)
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
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        self.protocol.log("info", "disconnected. " + str(cls.clients_connected) + " client total." )

    def send_message(self, message=None):
        """
        Callback from rosbridge
        """
        self.request.send(message)
