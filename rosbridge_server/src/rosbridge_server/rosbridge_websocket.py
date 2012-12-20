from roslib import load_manifest
load_manifest("rosbridge_server")
from rospy import init_node, get_param, loginfo, logerr

from functools import partial

from tornado.websocket import WebSocketHandler

from rosbridge_library.rosbridge_protocol import RosbridgeProtocol

# Global ID seed for clients
client_id_seed = 0
clients_connected = 0

class RosbridgeWebSocket(WebSocketHandler):

    def __init__(self):
        init_node("rosbridge_server")

    def open(self):
        global client_id_seed, clients_connected
        try:
            self.protocol = RosbridgeProtocol(client_id_seed)
            self.protocol.outgoing = self.send_message
            client_id_seed = client_id_seed + 1
            clients_connected = clients_connected + 1
        except Exception as exc:
            logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
#            raise
        loginfo("Client connected.  %d clients total.", clients_connected)

    def on_message(self, message):
        self.protocol.incoming(message)

    def on_close(self):
        global clients_connected
        clients_connected = clients_connected - 1
        self.protocol.finish()
        loginfo("Client disconnected.  %d clients total.", clients_connected)

    def send_message(self, message):
        IOLoop.instance().add_callback(partial(self.write_message, message))

    def allow_draft76(self):
        """Override to enable support for the older "draft76" protocol.

        The draft76 version of the websocket protocol is disabled by
        default due to security concerns, but it can be enabled by
        overriding this method to return True.

        Connections using the draft76 protocol do not support the
        ``binary=True`` flag to `write_message`.

        Support for the draft76 protocol is deprecated and will be
        removed in a future version of Tornado.
        """
        return True

