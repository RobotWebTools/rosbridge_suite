#!/usr/bin/env python
"""
Rosbridge client mode
"""

import time
from socket import error

import rospy
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.capabilities.call_service import CallService

from rosbridge_client import RosbridgeWebSocketClient


def shutdown_hook():
    pass


def main():
    rospy.init_node('rosbridge_websocket_client')
    rospy.on_shutdown(shutdown_hook)

    retry_startup_delay = rospy.get_param('~retry_startup_delay',
                                          2.0)  # seconds

    # get RosbridgeProtocol parameters
    RosbridgeWebSocketClient.fragment_timeout = rospy.get_param(
        '~fragment_timeout', RosbridgeWebSocketClient.fragment_timeout)
    RosbridgeWebSocketClient.delay_between_messages = rospy.get_param(
        '~delay_between_messages',
        RosbridgeWebSocketClient.delay_between_messages)
    RosbridgeWebSocketClient.max_message_size = rospy.get_param(
        '~max_message_size', RosbridgeWebSocketClient.max_message_size)
    RosbridgeWebSocketClient.unregister_timeout = rospy.get_param(
        '~unregister_timeout', RosbridgeWebSocketClient.unregister_timeout)
    bson_only_mode = rospy.get_param('~bson_only_mode', False)

    if RosbridgeWebSocketClient.max_message_size == "None":
        RosbridgeWebSocketClient.max_message_size = None

    # SSL options
    certfile = rospy.get_param('~certfile', None)
    keyfile = rospy.get_param('~keyfile', None)

    # Get the glob strings and parse them as arrays.
    RosbridgeWebSocketClient.topics_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~topics_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0
    ]
    RosbridgeWebSocketClient.services_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~services_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0
    ]
    RosbridgeWebSocketClient.params_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~params_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0
    ]

    # To be able to access the list of topics and services, you must be able to access the rosapi services.
    if RosbridgeWebSocketClient.services_glob:
        RosbridgeWebSocketClient.services_glob.append("/rosapi/*")

    Subscribe.topics_glob = RosbridgeWebSocketClient.topics_glob
    Advertise.topics_glob = RosbridgeWebSocketClient.topics_glob
    Publish.topics_glob = RosbridgeWebSocketClient.topics_glob
    AdvertiseService.services_glob = RosbridgeWebSocketClient.services_glob
    UnadvertiseService.services_glob = RosbridgeWebSocketClient.services_glob
    CallService.services_glob = RosbridgeWebSocketClient.services_glob

    address = rospy.get_param('~address', "ws://localhost:9090")

    connected = False
    ws = None
    while not connected and not rospy.is_shutdown():
        try:
            if certfile is not None and keyfile is not None:
                ws = RosbridgeWebSocketClient(
                    address,
                    ssl_options={
                        "certfile": certfile,
                        "keyfile": keyfile
                    })
                ws.connect()
            else:
                ws = RosbridgeWebSocketClient(address)
                ws.connect()
            rospy.loginfo("Rosbridge WebSocket connected on %s" % address)
            connected = True
        except error as e:
            rospy.logwarn("Unable to access server: " + str(e) +
                          " Retrying in " + str(retry_startup_delay) + "s.")
            rospy.sleep(retry_startup_delay)

    r = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        r.sleep()

    rospy.loginfo('Bye Bye')
    ws.close()


if __name__ == '__main__':
    main()
