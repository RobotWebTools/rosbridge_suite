import rospy
from rosbridge_library.rosbridge_protocol import RosbridgeProtocol
from rosbridge_library.util import json
import bson

from twisted.internet.protocol import DatagramProtocol

class UDPHandler(DatagramProtocol):
    client_id_seed = 0
    clients_connected = 0
    authenticate = False

    def startProtocol(self):
        cls = self.__class__
        try:
            self.protocol = RosbridgeProtocol(cls.client_id_seed)
            self.protocol.outgoing = self.send_message
            self.set_nodelay(True)
            self.authenticated = False
            cls.client_id_seed += 1
            cls.clients_connected += 1
        except Exception as exc:
            rospy.logerr("Unable to accept incoming connection.  Reason: %s", str(exc))
        rospy.loginfo("Client connected.  %d clients total.", cls.clients_connected)
        if cls.authenticate:
            rospy.loginfo("Awaiting proper authentication...")

    def datagramReceived(self, message, (host, port)):
        print "received %r from %s:%d" % (data, host, port)
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
        print "sending {0}".format(message)
        self.transport.write(message)
    def check_origin(self, origin):
        return False