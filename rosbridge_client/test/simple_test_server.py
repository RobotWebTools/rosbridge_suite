#!/usr/bin/env python

from __future__ import print_function

from socket import error
import time
from functools import partial

from tornado.ioloop import IOLoop
from tornado.ioloop import PeriodicCallback
from tornado.web import Application

from tornado.ioloop import IOLoop
from tornado.websocket import WebSocketHandler
import threading

import json
import bson

class Handler(WebSocketHandler):

    def __init__(self, *args, **kwarg):
        super(Handler, self).__init__(*args, **kwarg)
        self._num_client = 0
        self._lock = threading.Lock()

    def open(self):
        with self._lock:
            self._num_client += 1
            print("Open. Num Client = %s" % self._num_client)

        self._advertise()
        time.sleep(1.0)
        self._publish()
        time.sleep(1.0)
        # self._subscribe()

    def on_message(self, message):
        print("Received %s" % message)

    def on_close(self):
        with self._lock:
            self._num_client -= 1
            print("Closed. Num Client = %s " % self._num_client)

    def send_message(self, message):
        binary = type(message)==bson.BSON
        IOLoop.instance().add_callback(
            partial(self.write_message, message, binary))

    def check_origin(self, origin):
        return True

    def _advertise(self):
        msg = {
            "op": "advertise",
            "topic": "/take_off",
            "type": "std_msgs/String"
        }
        self.send_message(json.dumps(msg))
        print("Advertise %s" % msg)

    def _publish(self):
        msg = { "op": "publish",
                "topic": "/take_off",
                "msg": {
                    "data": "Take off!!"
                }
            }
        self.send_message(json.dumps(msg))
        print("Publish %s" % msg)


def main():
    app = Application([(r"/", Handler), (r"", Handler)])
    connected = False
    address = 'localhost'
    port = 9090

    num_try = 0
    while not connected and num_try < 5:
        try:
            app.listen(port, address)
            print("Started on port %d" % port)
            connected = True
        except error as e:
            print("Failed to start server retry...")
            time.sleep(2.0)
            num_try += 1
    IOLoop.instance().start()


if __name__ == '__main__':
    main()
