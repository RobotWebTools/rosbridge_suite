#import rospy
import time
from ws4py.client.threadedclient import WebSocketClient

#from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
#from rosbridge_library.util import json, bson


class RosbridgeWebSocketClient(WebSocketClient):
    """
    Connects to exisiting webserver to wait for incoming data from user.

    This is minimal implementation which may not have full feature.
    """

    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600  # seconds
    # protocol.py:
    delay_between_messages = 0  # seconds
    max_message_size = None  # bytes
    unregister_timeout = 10.0  # seconds
    bson_only_mode = False
    seed = 0

    def opened(self):
        """
        Called by the server when the handshake is successful
        """
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout,
            "bson_only_mode": cls.bson_only_mode
        }
        try:
            self.protocol = RosbridgeProtocol(cls.seed, parameters=parameters)
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            self.authenticated = False
        except Exception as exc:
            pass
            rospy.logerr("Unable to accept incoming connection.  Reason: %s",
                         str(exc))
        # rospy.loginfo("connected.")
        print("Conected")

    def closed(self):
        """
        Called by the server when the websocket stream and connection are
        finally closed
        """
        # rospy.loginfo("Closed")
        print("Closed")
        pass
        self.protocol.finish()

    def received_message(self, message):
        """
        Process incoming from server
        """
        pass
        self.protocol.incoming(message)

    def send_message(self, message):
        """
        send message to server
        """
        binary = type(message) == bson.BSON
        self.send(message, binary)
