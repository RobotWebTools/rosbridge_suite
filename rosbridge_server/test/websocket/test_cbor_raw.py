#!/usr/bin/env python
import json
import io
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

from rosbridge_library.util.cbor import loads as decode_cbor

from twisted.python import log
log.startLogging(sys.stderr)

TOPIC = '/b_topic'
STRING = 'B' * 10000
WARMUP_DELAY = 1.0  # seconds
TIME_LIMIT = 5.0  # seconds


class TestWebsocketCborRaw(unittest.TestCase):
    def test_cbor_raw(self):
        test_client_received = []
        class TestClientProtocol(WebSocketClientProtocol):
            def onOpen(self):
                self.sendMessage(json.dumps({
                    'op': 'subscribe',
                    'topic': TOPIC,
                    'compression': 'cbor-raw',
                }).encode('utf-8'))

            def onMessage(self, payload, binary):
                test_client_received.append(payload)

        protocol = os.environ.get('PROTOCOL')
        port = rospy.get_param('/rosbridge_websocket/actual_port')
        url = protocol + '://127.0.0.1:' + str(port)

        factory = WebSocketClientFactory(url)
        factory.protocol = TestClientProtocol
        reactor.connectTCP('127.0.0.1', port, factory)

        pub = rospy.Publisher(TOPIC, String, queue_size=1)
        def publish_timer():
            rospy.sleep(WARMUP_DELAY)
            pub.publish(String(STRING))
            rospy.sleep(TIME_LIMIT)
            reactor.stop()
        reactor.callInThread(publish_timer)
        reactor.run()

        self.assertEqual(len(test_client_received), 1)
        websocket_message = decode_cbor(test_client_received[0])
        self.assertEqual(websocket_message['topic'], TOPIC)
        buff = io.BytesIO()
        String(STRING).serialize(buff)
        self.assertEqual(websocket_message['msg']['bytes'], buff.getvalue())


PKG = 'rosbridge_server'
NAME = 'test_websocket_cbor_raw'

if __name__ == '__main__':
    rospy.init_node(NAME)

    while not rospy.is_shutdown() and not rospy.has_param('/rosbridge_websocket/actual_port'):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestWebsocketCborRaw)
