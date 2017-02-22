#!usr/bin/env python
import sys
import rospy
import bson
import json
from sensor_msgs.msg import CompressedImage
from websocket import create_connection
from rospy_message_converter import message_converter

ws = create_connection("ws://ineedcaffeine.xyz:9090")


def img_cb(data):

    ws_data = bson.BSON.encode({
        "op": "publish",
        "topic": "remote_cam/compressed",
        "msg": message_converter.convert_ros_message_to_dictionary(data)
    })
    ws.send_binary(ws_data)

if __name__ == "__main__":
    rospy.init_node('stream_node')
    # r = rospy.Rate(1);
    ws.send_binary(bson.BSON.encode({
        "op": "advertise",
        "topic": "remote_cam/compressed",
        "type": "sensor_msgs/CompressedImage"
    }))
    s = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, img_cb)

    rospy.spin()
