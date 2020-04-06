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

# For consistency, the number of messages must not exceed the the protocol
# Subscriber queue_size.
NUM_MSGS = 10
MSG_SIZE = 10
A_TOPIC = '/a_topic'
B_TOPIC = '/b_topic'
A_STRING = 'A' * MSG_SIZE
B_STRING = 'B' * MSG_SIZE
WARMUP_DELAY = 1.0  # seconds
TIME_LIMIT = 5.0  # seconds


class TestClientProtocol(WebSocketClientProtocol):
    def onOpen(self):
        self._sendDict({
            'op': 'subscribe',
            'topic': B_TOPIC,
            'type': 'std_msgs/String',
            'queue_length': 0,  # Test the roslib default.
        })
        self._sendDict({
            'op': 'advertise',
            'topic': A_TOPIC,
            'type': 'std_msgs/String',
        })
        self._sendDict({
            'op': 'publish',
            'topic': A_TOPIC,
            'msg': {
                'data': A_STRING,
            },
        }, NUM_MSGS)

    def _sendDict(self, msg_dict, times=1):
        msg = json.dumps(msg_dict).encode('utf-8')
        for _ in range(times):
            self.sendMessage(msg)

    def onMessage(self, payload, binary):
        self.__class__.received.append(payload)


class TestWebsocketSmoke(unittest.TestCase):
    def test_smoke(self):
        protocol = os.environ.get('PROTOCOL')
        port = rospy.get_param('/rosbridge_websocket/actual_port')
        url = protocol + '://127.0.0.1:' + str(port)

        factory = WebSocketClientFactory(url)
        factory.protocol = TestClientProtocol
        reactor.connectTCP('127.0.0.1', port, factory)

        ros_received = []
        TestClientProtocol.received = []
        rospy.Subscriber(A_TOPIC, String, ros_received.append)
        pub = rospy.Publisher(B_TOPIC, String, queue_size=NUM_MSGS)

        def publish_timer():
            rospy.sleep(WARMUP_DELAY)
            msg = String(B_STRING)
            for _ in range(NUM_MSGS):
                pub.publish(msg)

        def shutdown_timer():
            rospy.sleep(WARMUP_DELAY + TIME_LIMIT)
            reactor.stop()

        reactor.callInThread(publish_timer)
        reactor.callInThread(shutdown_timer)
        reactor.run()

        for received in TestClientProtocol.received:
            msg = json.loads(received)
            self.assertEqual('publish', msg['op'])
            self.assertEqual(B_TOPIC, msg['topic'])
            self.assertEqual(B_STRING, msg['msg']['data'])
        self.assertEqual(NUM_MSGS, len(TestClientProtocol.received))

        for msg in ros_received:
            self.assertEqual(A_STRING, msg.data)
        self.assertEqual(NUM_MSGS, len(ros_received))


PKG = 'rosbridge_server'
NAME = 'test_websocket_smoke'

if __name__ == '__main__':
    rospy.init_node(NAME)

    while not rospy.is_shutdown() and not rospy.has_param('/rosbridge_websocket/actual_port'):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestWebsocketSmoke)
