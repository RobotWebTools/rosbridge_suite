import rospy
import bson
from rosbridge_library.protocol import Protocol
from rospy_message_converter import message_converter
from tornado.websocket import WebSocketHandler
import websocket


class RosbridgeClient(WebSocketHandler):
    """
    Rosbridge Client Class
    for python client working with rosbridge
    """

    ws = None

    def __init__(self):
        addr = None
        # try:
        #     self.ws = websocket.create_connection("ws://" + addr + ":9090")
        # except Exception as exc:
        #     rospy.loginfo("create connection fail:"+str(exc))

    def connect(self, addr):
        self.ws = websocket.create_connection("ws://147.46.242.59:9090")

    def advertise(self, topic, tp):
        adv = {
            "op": "advertise",
            "topic": topic,
            "type": tp
        }
        self.ws.send_binary(bson.BSON.encode(adv))


    def publish(self, topic, msg):
        msg_pydict = message_converter.convert_ros_message_to_dictionary(msg)
        pub = {
            "op": "publish",
            "topic": topic,
            "data": msg_pydict
        }
        self.ws.send_binary(bson.BSON.encode(pub))

    def subscribe(self, topic, callback):
        sub_msg = {
            "op": "subscribe",
            "topic": topic
        }
        self.ws.send_binary(bson.BSON.encode(sub_msg))

    def open(self):
        rospy.loginfo("connected to server")

    def on_message(self, message):
        print(bson.BSON(message))

    def on_close(self):
        rospy.loginfo("connection terminated")
