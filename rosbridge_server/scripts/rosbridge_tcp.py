#!/usr/bin/env python

from __future__ import print_function
from rospy import init_node, get_param, loginfo, logerr, on_shutdown, Publisher
from rosbridge_server import RosbridgeTcpSocket

from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.capabilities.call_service import CallService

from functools import partial
from signal import signal, SIGINT, SIG_DFL
from std_msgs.msg import Int32

try:
    import SocketServer
except ImportError:
    import socketserver as SocketServer

import sys
import time

#TODO: take care of socket timeouts and make sure to close sockets after killing programm to release network ports

#TODO: add new parameters to websocket version! those of rosbridge_tcp.py might not be needed, but the others should work well when adding them to .._websocket.py


def shutdown_hook(server):
        server.shutdown()

if __name__ == "__main__":
    loaded = False
    retry_count = 0
    while not loaded:
        retry_count += 1
        print("trying to start rosbridge TCP server..")
        try:
            print("")
            init_node("rosbridge_tcp")
            signal(SIGINT, SIG_DFL)

            """
            Parameter handling:
             - try to get parameter from parameter server (..define those via launch-file)
             - overwrite value if given as commandline-parameter

            BEGIN...
            """

#TODO: ensure types get cast correctly after getting from parameter server
#TODO: check if ROS parameter server uses None string for 'None-value' or Null or something else, then change code accordingly

            # update parameters from parameter server or use default value ( second parameter of get_param )
            port = get_param('~port', 9090)
            host = get_param('~host', '')
            incoming_buffer = get_param('~incoming_buffer', RosbridgeTcpSocket.incoming_buffer)
            socket_timeout = get_param('~socket_timeout', RosbridgeTcpSocket.socket_timeout)
            retry_startup_delay = get_param('~retry_startup_delay', 5.0)  # seconds
            fragment_timeout = get_param('~fragment_timeout', RosbridgeTcpSocket.fragment_timeout)
            delay_between_messages = get_param('~delay_between_messages', RosbridgeTcpSocket.delay_between_messages)
            max_message_size = get_param('~max_message_size', RosbridgeTcpSocket.max_message_size)
            unregister_timeout = get_param('~unregister_timeout', RosbridgeTcpSocket.unregister_timeout)
            bson_only_mode = get_param('~bson_only_mode', False)

            if max_message_size == "None":
                max_message_size = None

            # Get the glob strings and parse them as arrays.
            RosbridgeTcpSocket.topics_glob = [
                    element.strip().strip("'")
                    for element in get_param('~topics_glob', '')[1:-1].split(',')
                    if len(element.strip().strip("'")) > 0]
            RosbridgeTcpSocket.services_glob = [
                    element.strip().strip("'")
                    for element in get_param('~services_glob', '')[1:-1].split(',')
                    if len(element.strip().strip("'")) > 0]
            RosbridgeTcpSocket.params_glob = [
                    element.strip().strip("'")
                    for element in get_param('~params_glob', '')[1:-1].split(',')
                    if len(element.strip().strip("'")) > 0]
            
            # Publisher for number of connected clients
            RosbridgeTcpSocket.client_count_pub = Publisher('client_count', Int32, queue_size=10, latch=True)
            RosbridgeTcpSocket.client_count_pub.publish(0)

            # update parameters if provided via commandline
            # .. could implemented 'better' (value/type checking, etc.. )
            if "--port" in sys.argv:
                idx = sys.argv.index("--port") + 1
                if idx < len(sys.argv):
                    port = int(sys.argv[idx])
                else:
                    print("--port argument provided without a value.")
                    sys.exit(-1)

            if "--host" in sys.argv:
                idx = sys.argv.index("--host") + 1
                if idx < len(sys.argv):
                    host = str(sys.argv[idx])
                else:
                    print("--host argument provided without a value.")
                    sys.exit(-1)

            if "--incoming_buffer" in sys.argv:
                idx = sys.argv.index("--incoming_buffer") + 1
                if idx < len(sys.argv):
                    incoming_buffer = int(sys.argv[idx])
                else:
                    print("--incoming_buffer argument provided without a value.")
                    sys.exit(-1)

            if "--socket_timeout" in sys.argv:
                idx = sys.argv.index("--socket_timeout") + 1
                if idx < len(sys.argv):
                    socket_timeout = int(sys.argv[idx])
                else:
                    print("--socket_timeout argument provided without a value.")
                    sys.exit(-1)

            if "--retry_startup_delay" in sys.argv:
                idx = sys.argv.index("--retry_startup_delay") + 1
                if idx < len(sys.argv):
                    retry_startup_delay = int(sys.argv[idx])
                else:
                    print("--retry_startup_delay argument provided without a value.")
                    sys.exit(-1)

            if "--fragment_timeout" in sys.argv:
                idx = sys.argv.index("--fragment_timeout") + 1
                if idx < len(sys.argv):
                    fragment_timeout = int(sys.argv[idx])
                else:
                    print("--fragment_timeout argument provided without a value.")
                    sys.exit(-1)

            if "--delay_between_messages" in sys.argv:
                idx = sys.argv.index("--delay_between_messages") + 1
                if idx < len(sys.argv):
                    delay_between_messages = float(sys.argv[idx])
                else:
                    print("--delay_between_messages argument provided without a value.")
                    sys.exit(-1)

            if "--max_message_size" in sys.argv:
                idx = sys.argv.index("--max_message_size") + 1
                if idx < len(sys.argv):
                    value = sys.argv[idx]
                    if value == "None":
                        max_message_size = None
                    else:
                        max_message_size = int(value)
                else:
                    print("--max_message_size argument provided without a value. (can be None or <Integer>)")
                    sys.exit(-1)

            if "--unregister_timeout" in sys.argv:
                idx = sys.argv.index("--unregister_timeout") + 1
                if idx < len(sys.argv):
                    unregister_timeout = float(sys.argv[idx])
                else:
                    print("--unregister_timeout argument provided without a value.")
                    sys.exit(-1)

            # export parameters to handler class
            RosbridgeTcpSocket.incoming_buffer = incoming_buffer
            RosbridgeTcpSocket.socket_timeout = socket_timeout
            RosbridgeTcpSocket.fragment_timeout = fragment_timeout
            RosbridgeTcpSocket.delay_between_messages = delay_between_messages
            RosbridgeTcpSocket.max_message_size = max_message_size
            RosbridgeTcpSocket.unregister_timeout = unregister_timeout
            RosbridgeTcpSocket.bson_only_mode = bson_only_mode


            if "--topics_glob" in sys.argv:
                idx = sys.argv.index("--topics_glob") + 1
                if idx < len(sys.argv):
                    value = sys.argv[idx]
                    if value == "None":
                        RosbridgeTcpSocket.topics_glob = []
                    else:
                        RosbridgeTcpSocket.topics_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
                else:
                    print("--topics_glob argument provided without a value. (can be None or a list)")
                    sys.exit(-1)

            if "--services_glob" in sys.argv:
                idx = sys.argv.index("--services_glob") + 1
                if idx < len(sys.argv):
                    value = sys.argv[idx]
                    if value == "None":
                        RosbridgeTcpSocket.services_glob = []
                    else:
                        RosbridgeTcpSocket.services_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
                else:
                    print("--services_glob argument provided without a value. (can be None or a list)")
                    sys.exit(-1)

            if "--params_glob" in sys.argv:
                idx = sys.argv.index("--params_glob") + 1
                if idx < len(sys.argv):
                    value = sys.argv[idx]
                    if value == "None":
                        RosbridgeTcpSocket.params_glob = []
                    else:
                        RosbridgeTcpSocket.params_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
                else:
                    print("--params_glob argument provided without a value. (can be None or a list)")
                    sys.exit(-1)

            if "--bson_only_mode" in sys.argv:
                bson_only_mode = True

            # To be able to access the list of topics and services, you must be able to access the rosapi services.
            if RosbridgeTcpSocket.services_glob:
                RosbridgeTcpSocket.services_glob.append("/rosapi/*")

            Subscribe.topics_glob = RosbridgeTcpSocket.topics_glob
            Advertise.topics_glob = RosbridgeTcpSocket.topics_glob
            Publish.topics_glob = RosbridgeTcpSocket.topics_glob
            AdvertiseService.services_glob = RosbridgeTcpSocket.services_glob
            UnadvertiseService.services_glob = RosbridgeTcpSocket.services_glob
            CallService.services_glob = RosbridgeTcpSocket.services_glob

            """
            ...END (parameter handling)
            """


            # Server host is a tuple ('host', port)
            # empty string for host makes server listen on all available interfaces
            SocketServer.ThreadingTCPServer.allow_reuse_address = True
            server = SocketServer.ThreadingTCPServer((host, port), RosbridgeTcpSocket)
            on_shutdown(partial(shutdown_hook, server))

            loginfo("Rosbridge TCP server started on port %d", port)

            server.serve_forever()
            loaded = True
        except Exception as e:
            time.sleep(retry_startup_delay)
    print("server loaded")
