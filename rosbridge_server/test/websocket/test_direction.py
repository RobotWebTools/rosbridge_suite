#!/usr/bin/env python
import json
import os
import rostest
import sys
import threading
import unittest

import rospy
from std_msgs.msg import String

from twisted.internet import reactor
from autobahn.twisted.websocket import (WebSocketClientProtocol,
                                        WebSocketClientFactory)

from twisted.python import log
log.startLogging(sys.stderr)

# This unit test tests whether the messages are appropriately gated.
# The client attempts to publish on the OUTGOING_TOPIC topic. This should not go through.
# The client attempts to receive on the  INGOING_TOPIC topic. This should not go through.
# The client attempts to publish on the  INGOING_TOPIC topic. This should go through.
# The client attempts to receive on the OUTGOING_TOPIC topic. This should go through.


# For consistency, the number of messages must not exceed the the protocol
# Subscriber queue_size.
NUM_MSGS = 10
OUTGOING_TOPIC = '/outgoing_only'
INGOING_TOPIC = '/ingoing_only'

CLIENT_PUB_TO_OUTGOING = "CLIENT_PUB_TO_OUTGOING"
SERVER_PUB_TO_OUTGOING = "SERVER_PUB_TO_OUTGOING"
CLIENT_PUB_TO_INGOING = "CLIENT_PUB_TO_INGOING"
SERVER_PUB_TO_INGOING = "SERVER_PUB_TO_INGOING"
# Client should only receive SERVER_PUB_TO_OUTGOING
# Server should only receive CLIENT_PUB_TO_INGOING and SERVER_PUB_TO_INGOING (loopback)

# The bidirectional topics are tested by the test_smoke.py
WARMUP_DELAY = 1.0  # seconds
TIME_LIMIT = 5.0  # seconds

class TestClientProtocol(WebSocketClientProtocol):
    def onOpen(self):
        # client attempt to publish on outgoing.
        self._sendDict({
            'op': 'advertise',
            'topic': OUTGOING_TOPIC,
            'type': 'std_msgs/String',
        })
        self._sendDict({
            'op': 'publish',
            'topic': OUTGOING_TOPIC,
            'msg': {
                'data': CLIENT_PUB_TO_OUTGOING,
            },
        }, NUM_MSGS)

        # client attempt to subscribe to outgoing.
        self._sendDict({
            'op': 'subscribe',
            'topic': OUTGOING_TOPIC,
            'type': 'std_msgs/String',
            'queue_length': 0,
        })

        # client attempt to subscribe on outgoing.
        self._sendDict({
            'op': 'subscribe',
            'topic': INGOING_TOPIC,
            'type': 'std_msgs/String',
            'queue_length': 0,
        })
        # Client attempt to publish on the ingoing topic.
        self._sendDict({
            'op': 'advertise',
            'topic': INGOING_TOPIC,
            'type': 'std_msgs/String',
        })
        self._sendDict({
            'op': 'publish',
            'topic': INGOING_TOPIC,
            'msg': {
                'data': CLIENT_PUB_TO_INGOING,
            },
        }, NUM_MSGS)


    def _sendDict(self, msg_dict, times=1):
        msg = json.dumps(msg_dict).encode('utf-8')
        for _ in range(times):
            self.sendMessage(msg)

    def onMessage(self, payload, binary):
        self.__class__.received.append(payload)


class TestWebsocketDirection(unittest.TestCase):
    def test_smoke(self):
        protocol = os.environ.get('PROTOCOL')
        port = rospy.get_param('/rosbridge_websocket/actual_port')
        url = protocol + '://127.0.0.1:' + str(port)

        factory = WebSocketClientFactory(url)
        factory.protocol = TestClientProtocol
        reactor.connectTCP('127.0.0.1', port, factory)

        ros_received = []
        TestClientProtocol.received = []
        rospy.Subscriber(INGOING_TOPIC, String, ros_received.append)
        pub_outgoing = rospy.Publisher(OUTGOING_TOPIC, String, queue_size=NUM_MSGS)
        pub_ingoing = rospy.Publisher(INGOING_TOPIC, String, queue_size=NUM_MSGS)
        

        def publish_outgoing_timer():
            rospy.sleep(WARMUP_DELAY)
            msg = String(SERVER_PUB_TO_OUTGOING)
            for _ in range(NUM_MSGS):
                pub_outgoing.publish(msg)

        def publish_ingoing_timer():
            rospy.sleep(WARMUP_DELAY)
            msg = String(SERVER_PUB_TO_INGOING)
            for _ in range(NUM_MSGS):
                pub_ingoing.publish(msg)

        def shutdown_timer():
            rospy.sleep(WARMUP_DELAY + TIME_LIMIT)
            reactor.stop()

        reactor.callInThread(publish_outgoing_timer)
        reactor.callInThread(publish_ingoing_timer)
        reactor.callInThread(shutdown_timer)
        reactor.run()

        # Done sending all the messages and receiving them, lets see if
        # the client and server obtained the right ones.

        # The client should have obtained NUM_MSGS only, namely the ones sent   
        # by the server on the outgoing topic.
        self.assertEqual(NUM_MSGS, len(TestClientProtocol.received))
        for received in TestClientProtocol.received:
            msg = json.loads(received)
            self.assertEqual('publish', msg['op'])
            self.assertEqual(OUTGOING_TOPIC, msg['topic'])
            self.assertEqual(SERVER_PUB_TO_OUTGOING, msg['msg']['data'])        


        # Confirm that the server only obtained NUM_MSGS, on the server side
        # we have a loopback, since what we publish also ends up in the
        # subscription.
        self.assertEqual(NUM_MSGS * 2, len(ros_received))
        for msg in ros_received:
            allowed_receipt = {CLIENT_PUB_TO_INGOING, SERVER_PUB_TO_INGOING}
            self.assertTrue(msg.data in allowed_receipt)


PKG = 'rosbridge_server'
NAME = 'test_websocket_direction'

if __name__ == '__main__':
    rospy.init_node(NAME)

    while not rospy.is_shutdown() and not rospy.has_param('/rosbridge_websocket/actual_port'):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestWebsocketDirection)
