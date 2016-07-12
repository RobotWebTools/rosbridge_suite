import rospy
from std_msgs.msg import String
from rosbridge_client.client import RosbridgeClient


def callback_test(msg):
    rospy.loginfo(msg)

rospy.init_node("example_publisher")
r = rospy.Rate(10)

msg = String(data="hello")

ros_server = RosbridgeClient()
ros_server.connect("147.46.242.59")

ros_server.advertise("std_msgs/String", "/chatter")
ros_server.subscribe("std_msgs/String", "/chatter2", callback_test)

while not rospy.is_shutdown():
    ros_server.publish("/chatter", msg)
    r.sleep()
