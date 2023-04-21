from std_msgs.msg import Int32
from rosbridge_library.util import bson
import struct
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
import traceback

try:
    import SocketServer
except ImportError:
    import socketserver as SocketServer

class RosbridgeTcpSocket(SocketServer.BaseRequestHandler):
    """
    TCP Socket server for rosbridge
    """

    busy = False
    queue = []
    client_id_seed = 0
    clients_connected = 0
    client_count_pub = None

    # list of parameters
    incoming_buffer = 65536                 # bytes
    socket_timeout = 10                     # seconds
    # The following are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes
    unregister_timeout = 10.0               # seconds
    bson_only_mode = False
    ros_node = None

    def setup(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size,
            "unregister_timeout": cls.unregister_timeout,
            "bson_only_mode": cls.bson_only_mode
        }
        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, cls.ros_node, parameters=parameters)
            self.protocol.outgoing = self.send_message
            cls.client_id_seed += 1
            cls.clients_connected += 1
            if cls.client_count_pub:
                cls.client_count_pub.publish(Int32(data=cls.clients_connected))
            self.protocol.log("info", "connected. " + str(cls.clients_connected) + " client total.")
            self.force_close_request = False
        except Exception as exc:
            cls.ros_node.get_logger().info("Unable to accept incoming connection.  Reason: " + str(exc))

    def recvall(self, n):
        # http://stackoverflow.com/questions/17667903/python-socket-receive-large-amount-of-data
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytearray()
        while len(data) < n:
            packet = self.request.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def recv_bson(self):
        # Read 4 bytes to get the length of the BSON packet
        BSON_LENGTH_IN_BYTES = 4
        raw_msglen = self.recvall(BSON_LENGTH_IN_BYTES)
        if not raw_msglen:
            return None
        msglen = struct.unpack('i', raw_msglen)[0]
        
        # Retrieve the rest of the message
        data = self.recvall(msglen - BSON_LENGTH_IN_BYTES)
        if data is None:
            return None
        data = raw_msglen + data # Prefix the data with the message length that has already been received.
                                 # The message length is part of BSONs message format

        # Exit on empty message
        if len(data) == 0:
            return None
        
        self.protocol.incoming(data)
        return True

    def handle(self):
        """
        Listen for TCP messages
        """
        cls = self.__class__
        self.request.settimeout(cls.socket_timeout)
        while True:
            try:
                if self.force_close_request:
                    self.protocol.log("error", "Force close request was triggered due to exception on send_message(). Likely by ConnectionResetError or BrokenPipe. Terminating client connection.")
                    break

                if self.bson_only_mode:
                    if self.recv_bson() is None:
                        break
                    continue

                # non-BSON handling
                bytes = self.request.recv(cls.incoming_buffer)

                # Exit on empty message because socket was closed
                if len(bytes) == 0:
                    break
                else:
                    string_encoded = bytes.decode().strip()
                    if len(string_encoded) > 0:
                        self.protocol.incoming(string_encoded)
            except Exception:
                self.protocol.log("debug", "socket connection timed out! (ignore warning if client is only listening..)")

    def finish(self):
        """
        Called when TCP connection finishes
        Called after the handle() method to perform any clean-up actions required.
        If setup() raises an exception, this function will not be called.
        """
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        if cls.client_count_pub:
            cls.client_count_pub.publish(Int32(data=cls.clients_connected))
        self.protocol.log("info", "disconnected. " + str(cls.clients_connected) + " client total." )

    def send_message(self, message=None, compression="none"):
        """
        Callback from rosbridge
        """
        try:
            if isinstance(message, bson.BSON) or self.bson_only_mode:
                binary = True
            elif compression in ["cbor", "cbor-raw"]:
                binary = True
            else:
                binary = False

            if binary:
                self.request.sendall(message)
            else:
                self.request.sendall(message.encode())
        except Exception:
            self.force_close_request = True
            self.protocol.log("error", f"Exception during send_message(): {traceback.format_exc()}")
