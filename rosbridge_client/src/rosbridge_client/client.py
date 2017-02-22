import rospy
import bson
from rospy_message_converter import message_converter
import websocket
import threading
import time
from std_msgs.msg import String


class RosbridgeClient:
    """
    Rosbridge Client Class
    for python client working with rosbridge
    """

    ws = None
    subscribe_infos = []
    subscribe_num = 0

    def __init__(self):
        addr = None
        # try:
        #     self.ws = websocket.create_connection("ws://" + addr + ":9090")
        # except Exception as exc:
        #     rospy.loginfo("create connection fail:"+str(exc))

    def connect(self, addr):
        self.ws = websocket.create_connection("ws://" + addr + ":9090")
        t = threading.Thread(target=self.event_loop)
        t.start()

    def advertise(self, tp, topic):
        adv = {
            "op": "advertise",
            "topic": topic,
            "type": tp
        }
        self.ws.send_binary(bson.BSON.encode(adv))

    def subscribe(self, tp, topic, callback):
        sub_msg = {
            "op": "subscribe",
            "topic": topic
        }
        self.subscribe_infos.append({
            "id": self.subscribe_num,
            "topic": topic,
            "tp": tp,
            "callback": callback
        })
        self.subscribe_num += 1
        self.ws.send_binary(bson.BSON.encode(sub_msg))

    def publish(self, topic, message):
        if str(type(message)) == "<type 'dict'>":
            pub = {
                "op": "publish",
                "topic": topic,
                "msg": message
            }
        else:
            pub = {
                "op": "publish",
                "topic": topic,
                "msg": message_converter.convert_ros_message_to_dictionary(message)
            }
        self.ws.send_binary(bson.BSON.encode(pub))

    def on_message(self, message):
        callback = None
        msg_type = None
        msg_dict = bson.BSON.decode(bson.BSON(message))
        if msg_dict["op"] == "publish":
            for i in range(0, self.subscribe_infos.__len__()):
                if self.subscribe_infos[i]["topic"] == msg_dict["topic"]:
                    msg_type = self.subscribe_infos[i]["tp"]
                    callback = self.subscribe_infos[i]["callback"]
            callback(message_converter.convert_dictionary_to_ros_message(msg_type, msg_dict["msg"]))
        else:
            rospy.loginfo("some error occur")

    def on_close(self):
        rospy.loginfo("connection terminated")

    def event_loop(self):
        while True:
            msg = self.ws.recv()
            if msg == None:
                time.sleep(0)
            else:
                self.on_message(msg)
