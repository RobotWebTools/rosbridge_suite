#!/usr/bin/env python3
"""
Rosbridge client mode
"""

import time
from socket import error

"""
import rospy
from rosbridge_client import WebsocketHandler
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.capabilities.call_service import CallService
"""

from websocket_handler import RosbridgeWebSocketClient


def shutdown_hook():
    pass


def main():
    """
    rospy.init_node('rosbridge_websocket_client')
    rospy.on_shutdown(shutdown_hook)

    retry_startup_delay = rospy.get_param('~retry_startup_delay',
                                          2.0)  # seconds

    # get RosbridgeProtocol parameters
    WebsocketHandler.fragment_timeout = rospy.get_param(
        '~fragment_timeout', WebsocketHandler.fragment_timeout)
    WebsocketHandler.delay_between_messages = rospy.get_param(
        '~delay_between_messages', WebsocketHandler.delay_between_messages)
    WebsocketHandler.max_message_size = rospy.get_param(
        '~max_message_size', WebsocketHandler.max_message_size)
    WebsocketHandler.unregister_timeout = rospy.get_param(
        '~unregister_timeout', WebsocketHandler.unregister_timeout)
    bson_only_mode = rospy.get_param('~bson_only_mode', False)

    if WebsocketHandler.max_message_size == "None":
        WebsocketHandler.max_message_size = None

    # SSL options
    certfile = rospy.get_param('~certfile', None)
    keyfile = rospy.get_param('~keyfile', None)

    address = rospy.get_param('~address', "")
    # Get the glob strings and parse them as arrays.
    WebsocketHandler.topics_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~topics_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0
    ]
    WebsocketHandler.services_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~services_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0
    ]
    WebsocketHandler.params_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~params_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0
    ]

    # To be able to access the list of topics and services, you must be able to access the rosapi services.
    if WebsocketHandler.services_glob:
        WebsocketHandler.services_glob.append("/rosapi/*")

    Subscribe.topics_glob = WebsocketHandler.topics_glob
    Advertise.topics_glob = WebsocketHandler.topics_glob
    Publish.topics_glob = WebsocketHandler.topics_glob
    AdvertiseService.services_glob = WebsocketHandler.services_glob
    UnadvertiseService.services_glob = WebsocketHandler.services_glob
    CallService.services_glob = WebsocketHandler.services_glob
    """
    address = 'ws://whoola.local:9090/'
    ws = RosbridgeWebSocketClient(address)
    ws.connect()
    print("Connected")
    ws.run_forever()

    print("Here")
    time.sleep(10.0)
    """
    connected = False
    while not connected and not rospy.is_shutdown():
        try:
            if certfile is not None and keyfile is not None:
                ws.connect(
                    url=address,
                    ssl_options={
                        "certfile": certfile,
                        "keyfile": keyfile
                    })
            else:
                ws.connect(url=address)
            rospy.loginfo("Rosbridge WebSocket connected on %s" % address)
            connected = True
        except error as e:
            rospy.logwarn("Unable to access server: " + str(e) +
                          " Retrying in " + str(retry_startup_delay) + "s.")
            rospy.sleep(retry_startup_delay)

    ws.run_forever()
    """


if __name__ == '__main__':
    main()
