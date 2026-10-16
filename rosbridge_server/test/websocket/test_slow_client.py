#!/usr/bin/env python
"""A slow client that disconnects must not stall a topic for other clients.

The slow client subscribes, stops reading until backpressure closes its
outgoing valve, then closes the connection normally. The subscriber thread
shared by all clients must be released, so the fast client keeps receiving.
"""
import base64
import json
import os
import rostest
import socket
import struct
import sys
import threading
import time
import unittest

import rospy
from std_msgs.msg import String

from twisted.internet import reactor
from autobahn.twisted.websocket import (WebSocketClientProtocol,
                                        WebSocketClientFactory)

from twisted.python import log
log.startLogging(sys.stderr)

TOPIC = '/slow_client_topic'
MSG_SIZE = 20000  # bytes
PUBLISH_RATE = 50  # Hz, ~1 MB/s: fills the slow client's buffers quickly
WARMUP_DELAY = 1.0  # seconds
STALL_TIME = 5.0  # seconds the slow client stays connected without reading
GRACE_TIME = 1.0  # seconds after the slow client is gone
CHECK_TIME = 3.0  # seconds during which the fast client must receive


class FastClientProtocol(WebSocketClientProtocol):
    def onOpen(self):
        self.sendMessage(json.dumps({
            'op': 'subscribe',
            'topic': TOPIC,
            'type': 'std_msgs/String',
        }).encode('utf-8'))

    def onMessage(self, payload, binary):
        self.__class__.received_at.append(time.time())


def _send_frame(sock, opcode, payload):
    """Send a masked client frame."""
    mask = os.urandom(4)
    length = len(payload)
    if length < 126:
        header = struct.pack('!BB', 0x80 | opcode, 0x80 | length)
    else:
        header = struct.pack('!BBH', 0x80 | opcode, 0x80 | 126, length)
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    sock.sendall(header + mask + masked)


def run_slow_client(port, closed_event):
    """Subscribe, never read, then close normally once backpressure built up."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    sock.connect(('127.0.0.1', port))
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((
        'GET / HTTP/1.1\r\n'
        'Host: 127.0.0.1:{}\r\n'
        'Upgrade: websocket\r\n'
        'Connection: Upgrade\r\n'
        'Sec-WebSocket-Key: {}\r\n'
        'Sec-WebSocket-Version: 13\r\n\r\n'
    ).format(port, key).encode())
    response = b''
    while b'\r\n\r\n' not in response:
        response += sock.recv(1)
    _send_frame(sock, 0x1, json.dumps({
        'op': 'subscribe',
        'topic': TOPIC,
        'type': 'std_msgs/String',
    }).encode('utf-8'))
    time.sleep(STALL_TIME)
    _send_frame(sock, 0x8, struct.pack('!H', 1000))
    time.sleep(GRACE_TIME)
    sock.close()
    closed_event.set()


class TestWebsocketSlowClient(unittest.TestCase):
    def test_slow_client_close_does_not_stall_others(self):
        protocol = os.environ.get('PROTOCOL')
        port = rospy.get_param('/rosbridge_websocket/actual_port')
        url = protocol + '://127.0.0.1:' + str(port)

        factory = WebSocketClientFactory(url)
        factory.protocol = FastClientProtocol
        FastClientProtocol.received_at = []
        reactor.connectTCP('127.0.0.1', port, factory)

        pub = rospy.Publisher(TOPIC, String, queue_size=1)
        slow_client_closed = threading.Event()
        stop_publishing = threading.Event()
        check_window = {}

        def publish_timer():
            rospy.sleep(WARMUP_DELAY)
            msg = String('x' * MSG_SIZE)
            rate = rospy.Rate(PUBLISH_RATE)
            while not stop_publishing.is_set() and not rospy.is_shutdown():
                pub.publish(msg)
                rate.sleep()

        def slow_client_timer():
            rospy.sleep(WARMUP_DELAY / 2)
            run_slow_client(port, slow_client_closed)

        def shutdown_timer():
            slow_client_closed.wait()
            rospy.sleep(GRACE_TIME)
            check_window['start'] = time.time()
            rospy.sleep(CHECK_TIME)
            check_window['end'] = time.time()
            stop_publishing.set()
            reactor.stop()

        reactor.callInThread(publish_timer)
        reactor.callInThread(slow_client_timer)
        reactor.callInThread(shutdown_timer)
        reactor.run()

        received_in_window = [
            t for t in FastClientProtocol.received_at
            if check_window['start'] <= t <= check_window['end']
        ]
        self.assertGreater(
            len(received_in_window), 0,
            'fast client received nothing after the slow client disconnected')


PKG = 'rosbridge_server'
NAME = 'test_websocket_slow_client'

if __name__ == '__main__':
    rospy.init_node(NAME)

    while not rospy.is_shutdown() and not rospy.has_param('/rosbridge_websocket/actual_port'):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestWebsocketSlowClient)
