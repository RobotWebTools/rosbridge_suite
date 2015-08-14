#!/usr/bin/env python

from rospy import init_node, get_param, loginfo, logerr, on_shutdown
from rosbridge_server import RosbridgeTcpSocket

from functools import partial
from signal import signal, SIGINT, SIG_DFL

import SocketServer
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
        print "trying to start rosbridge TCP server.."
        try:
            print ""
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
            port = get_param('~port', RosbridgeTcpSocket.port)
            host = get_param('~host', RosbridgeTcpSocket.host)
            incoming_buffer = get_param('~incoming_buffer', RosbridgeTcpSocket.incoming_buffer)
            socket_timeout = get_param('~socket_timeout', RosbridgeTcpSocket.socket_timeout)
            retry_startup_delay = get_param('~retry_startup_delay', RosbridgeTcpSocket.retry_startup_delay)
            service_request_timeout = get_param('~service_request_timeout', RosbridgeTcpSocket.service_request_timeout)
            check_response_delay = get_param('~check_response_delay', RosbridgeTcpSocket.check_response_delay)
            wait_for_busy_service_provider = get_param('~wait_for_busy_service_provider', RosbridgeTcpSocket.wait_for_busy_service_provider)
            max_service_requests = get_param('~max_service_requests', RosbridgeTcpSocket.max_service_requests)
            fragment_timeout = get_param('~fragment_timeout', RosbridgeTcpSocket.fragment_timeout)
            delay_between_messages = get_param('~delay_between_messages', RosbridgeTcpSocket.delay_between_messages)
            max_message_size = get_param('~max_message_size', RosbridgeTcpSocket.max_message_size)
            if max_message_size == "None":
                max_message_size = None


            # update parameters if provided via commandline
            # .. could implemented 'better' (value/type checking, etc.. )
            if "--port" in sys.argv:
                idx = sys.argv.index("--port") + 1
                if idx < len(sys.argv):
                    port = int(sys.argv[idx])
                else:
                    print "--port argument provided without a value."
                    sys.exit(-1)

            if "--host" in sys.argv:
                idx = sys.argv.index("--host") + 1
                if idx < len(sys.argv):
                    host = str(sys.argv[idx])
                else:
                    print "--host argument provided without a value."
                    sys.exit(-1)

            if "--incoming_buffer" in sys.argv:
                idx = sys.argv.index("--incoming_buffer") + 1
                if idx < len(sys.argv):
                    incoming_buffer = int(sys.argv[idx])
                else:
                    print "--incoming_buffer argument provided without a value."
                    sys.exit(-1)

            if "--socket_timeout" in sys.argv:
                idx = sys.argv.index("--socket_timeout") + 1
                if idx < len(sys.argv):
                    socket_timeout = int(sys.argv[idx])
                else:
                    print "--socket_timeout argument provided without a value."
                    sys.exit(-1)

            if "--retry_startup_delay" in sys.argv:
                idx = sys.argv.index("--retry_startup_delay") + 1
                if idx < len(sys.argv):
                    retry_startup_delay = int(sys.argv[idx])
                else:
                    print "--retry_startup_delay argument provided without a value."
                    sys.exit(-1)

            if "--service_request_timeout" in sys.argv:
                idx = sys.argv.index("--service_request_timeout") + 1
                if idx < len(sys.argv):
                    service_request_timeout = int(sys.argv[idx])
                else:
                    print "--service_request_timeout argument provided without a value."
                    sys.exit(-1)

            if "--check_response_delay" in sys.argv:
                idx = sys.argv.index("--check_response_delay") + 1
                if idx < len(sys.argv):
                    check_response_delay = float(sys.argv[idx])
                else:
                    print "--check_response_delay argument provided without a value."
                    sys.exit(-1)

            if "--wait_for_busy_service_provider" in sys.argv:
                idx = sys.argv.index("--wait_for_busy_service_provider") + 1
                if idx < len(sys.argv):
                    wait_for_busy_service_provider = float(sys.argv[idx])
                else:
                    print "--wait_for_busy_service_provider argument provided without a value."
                    sys.exit(-1)

            if "--max_service_requests" in sys.argv:
                idx = sys.argv.index("--max_service_requests") + 1
                if idx < len(sys.argv):
                    max_service_requests = int(sys.argv[idx])
                else:
                    print "--max_service_requests argument provided without a value."
                    sys.exit(-1)

            if "--fragment_timeout" in sys.argv:
                idx = sys.argv.index("--fragment_timeout") + 1
                if idx < len(sys.argv):
                    fragment_timeout = int(sys.argv[idx])
                else:
                    print "--fragment_timeout argument provided without a value."
                    sys.exit(-1)

            if "--delay_between_messages" in sys.argv:
                idx = sys.argv.index("--delay_between_messages") + 1
                if idx < len(sys.argv):
                    delay_between_messages = float(sys.argv[idx])
                else:
                    print "--delay_between_messages argument provided without a value."
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
                    print "--max_message_size argument provided without a value. (can be None or <Integer>)"
                    sys.exit(-1)

            # export parameters to handler class
            RosbridgeTcpSocket.port = port
            RosbridgeTcpSocket.host = host
            RosbridgeTcpSocket.incoming_buffer = incoming_buffer
            RosbridgeTcpSocket.socket_timeout = socket_timeout
            RosbridgeTcpSocket.retry_startup_delay = retry_startup_delay
            RosbridgeTcpSocket.service_request_timeout = service_request_timeout
            RosbridgeTcpSocket.check_response_delay = check_response_delay
            RosbridgeTcpSocket.wait_for_busy_service_provider = wait_for_busy_service_provider
            RosbridgeTcpSocket.max_service_requests = max_service_requests
            RosbridgeTcpSocket.fragment_timeout = fragment_timeout
            RosbridgeTcpSocket.delay_between_messages = delay_between_messages
            RosbridgeTcpSocket.max_message_size = max_message_size

            """
            ...END (parameter handling)
            """


            # Server host is a tuple ('host', port)
            # empty string for host makes server listen on all available interfaces
            server = SocketServer.ThreadingTCPServer((host, port), RosbridgeTcpSocket)
            on_shutdown(partial(shutdown_hook, server))

            loginfo("Rosbridge TCP server started on port %d", port)

            server.serve_forever()
            loaded = True
        except Exception, e:
            time.sleep(retry_startup_delay)
    print "server loaded"
