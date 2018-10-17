#!/usr/bin/env python3

from __future__ import print_function

from socket import error
import time

from tornado.ioloop import IOLoop
from tornado.ioloop import PeriodicCallback
from tornado.web import Application

from tornado.ioloop import IOLoop
from tornado.websocket import WebSocketHandler


class Handler(WebSocketHandler):
    def open(self):
        print("Open")

    def on_message(self, message):
        print("Received %s" % message)


    def on_close(self):
        print("Closed")

    def send_message(self, message):
        IOLoop.instance().add_callback(partial(self.write_message, message, binary))

def main():
    app = Application([(r"/", Handler), (r"",Handler)])
    connected = False
    address = 'whoola.local'
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
