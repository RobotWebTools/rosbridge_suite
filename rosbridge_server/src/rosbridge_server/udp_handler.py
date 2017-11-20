import rospy
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import json, bson

from twisted.internet.protocol import DatagramProtocol,Factory

class RosbridgeUdpFactory(DatagramProtocol):
    def startProtocol(self):
        self.socks = dict()
    def datagramReceived(self, message, source_addr):
        (host, port) = source_addr
        endpoint = host.__str__() + port.__str__()
        if endpoint in self.socks:
            self.socks[endpoint].datagramReceived(message)
        else:
            writefunc = lambda msg: self.transport.write(msg, (host,port))
            self.socks[endpoint] = RosbridgeUdpSocket(writefunc)
            self.socks[endpoint].startProtocol()
            self.socks[endpoint].datagramReceived(message)

class RosbridgeUdpSocket:
    client_id_seed = 0
    clients_connected = 0
    authenticate = False

    # The following parameters are passed on to RosbridgeProtocol
    # defragmentation.py:
    fragment_timeout = 600                  # seconds
    # protocol.py:
    delay_between_messages = 0              # seconds
    max_message_size = None                 # bytes

    def __init__(self,write):
        self.write = write
        self.authenticated = False

    def startProtocol(self):
        cls = self.__class__
        parameters = {
            "fragment_timeout": cls.fragment_timeout,
            "delay_between_messages": cls.delay_between_messages,
            "max_message_size": cls.max_message_size
        }
        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed, parameters=parameters)
            self.protocol.outgoing = self.send_message
            self.authenticated = False
            cls.client_id_seed += 1
            cls.clients_connected += 1
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        rospy.loginfo("Client connected.  %d clients total.", cls.clients_connected)
        if cls.authenticate:
            rospy.loginfo("Awaiting proper authentication...")

    def datagramReceived(self, message):
        cls = self.__class__
        # check if we need to authenticate
        if cls.authenticate and not self.authenticated:
            try:
                msg = json.loads(message)
                if msg['op'] == 'auth':
                    # check the authorization information
                    auth_srv = rospy.ServiceProxy('authenticate', Authentication)
                    resp = auth_srv(msg['mac'], msg['client'], msg['dest'],
                                                  msg['rand'], rospy.Time(msg['t']), msg['level'],
                                                  rospy.Time(msg['end']))
                    self.authenticated = resp.authenticated
                    if self.authenticated:
                        rospy.loginfo("Client %d has authenticated.", self.protocol.client_id)
                        return
                # if we are here, no valid authentication was given
                rospy.logwarn("Client %d did not authenticate. Closing connection.",
                              self.protocol.client_id)
                self.close()
            except:
                # proper error will be handled in the protocol class
                self.protocol.incoming(message)
        else:
            # no authentication required
            self.protocol.incoming(message)

    def stopProtocol(self):
        cls = self.__class__
        cls.clients_connected -= 1
        self.protocol.finish()
        rospy.loginfo("Client disconnected. %d clients total.", cls.clients_connected)
    def send_message(self, message):
        binary = type(message)==bson.BSON
        self.write(message)
    def check_origin(self, origin):
        return False
