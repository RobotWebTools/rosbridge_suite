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
from autobahn.websocket.util import create_url
from autobahn.twisted.websocket import (WebSocketClientProtocol,
                                        WebSocketClientFactory)

from twisted.python import log
log.startLogging(sys.stderr)

A_TOPIC = '/a_topic'
B_TOPIC = '/b_topic'
A_STRING = '''i'm trapped!'''
B_STRING = '''i'm free!'''


class TestClientProtocol(WebSocketClientProtocol):
    def onOpen(self):
        self._sendDict({
            'op': 'subscribe',
            'topic': B_TOPIC,
            'type': 'std_msgs/String',
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
        })

    def _sendDict(self, msg_dict):
        self.sendMessage(json.dumps(msg_dict))

    def onMessage(self, payload, binary):
        self.__class__.received.append(payload)


class TestWebsocketSmoke(unittest.TestCase):
    def test_smoke(self):
        port = int(os.environ.get('PORT'))
        url = create_url('127.0.0.1', port=port, isSecure=False)

        factory = WebSocketClientFactory(url)
        factory.protocol = TestClientProtocol
        reactor.connectTCP('127.0.0.1', port, factory)

        ros_received = []
        TestClientProtocol.received = []
        rospy.Subscriber(A_TOPIC, String, ros_received.append)
        pub = rospy.Publisher(B_TOPIC, String, queue_size=10)

        def publish_timer():
            rospy.sleep(1.5)
            pub.publish(String(B_STRING))

        def shutdown_timer():
            rospy.sleep(2.0)
            reactor.stop()

        rospy.sleep(1.0)  # wait for ROS subscriber connection

        reactor.callInThread(publish_timer)
        reactor.callInThread(shutdown_timer)
        reactor.run()

        self.assertEqual(1, len(TestClientProtocol.received))
        msg = json.loads(TestClientProtocol.received[0])
        self.assertEqual('publish', msg['op'])
        self.assertEqual(B_TOPIC, msg['topic'])
        self.assertEqual(B_STRING, msg['msg']['data'])

        self.assertEqual(1, len(ros_received))
        msg = ros_received[0]
        self.assertEqual(A_STRING, msg.data)


PKG = 'rosbridge_server'
NAME = 'test_websocket_smoke'

if __name__ == '__main__':
    rospy.init_node(NAME)

    rospy.sleep(3.0)  # server startup grace delay

    rostest.rosrun(PKG, NAME, TestWebsocketSmoke)
