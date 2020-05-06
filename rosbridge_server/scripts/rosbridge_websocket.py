#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import rospy
import sys

from twisted.python import log
from twisted.internet import reactor, ssl
from twisted.internet.error import CannotListenError, ReactorNotRunning
from distutils.version import LooseVersion
import autobahn #to check version
from autobahn.twisted.websocket import WebSocketServerFactory, listenWS
from autobahn.websocket.compress import (PerMessageDeflateOffer,
                                         PerMessageDeflateOfferAccept)
log.startLogging(sys.stdout)

from rosbridge_server import ClientManager
from rosbridge_server.autobahn_websocket import RosbridgeWebSocket
from rosbridge_server.util import get_ephemeral_port

from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.capabilities.call_service import CallService


def shutdown_hook():
    try:
        reactor.stop()
    except ReactorNotRunning:
        rospy.logwarn("Can't stop the reactor, it wasn't running")


if __name__ == "__main__":
    rospy.init_node("rosbridge_websocket")

    ##################################################
    # Parameter handling                             #
    ##################################################
    retry_startup_delay = rospy.get_param('~retry_startup_delay', 2.0)  # seconds

    use_compression = rospy.get_param('~use_compression', False)

    # get RosbridgeProtocol parameters
    RosbridgeWebSocket.fragment_timeout = rospy.get_param('~fragment_timeout',
                                                          RosbridgeWebSocket.fragment_timeout)
    RosbridgeWebSocket.delay_between_messages = rospy.get_param('~delay_between_messages',
                                                                RosbridgeWebSocket.delay_between_messages)
    RosbridgeWebSocket.max_message_size = rospy.get_param('~max_message_size',
                                                          RosbridgeWebSocket.max_message_size)
    RosbridgeWebSocket.unregister_timeout = rospy.get_param('~unregister_timeout',
                                                          RosbridgeWebSocket.unregister_timeout)
    bson_only_mode = rospy.get_param('~bson_only_mode', False)

    if RosbridgeWebSocket.max_message_size == "None":
        RosbridgeWebSocket.max_message_size = None

    ping_interval = float(rospy.get_param('~websocket_ping_interval', 0))
    ping_timeout = float(rospy.get_param('~websocket_ping_timeout', 30))
    null_origin = rospy.get_param('~websocket_null_origin', True) #default to original behaviour

    # SSL options
    certfile = rospy.get_param('~certfile', None)
    keyfile = rospy.get_param('~keyfile', None)
    # if authentication should be used
    RosbridgeWebSocket.authenticate = rospy.get_param('~authenticate', False)
    port = rospy.get_param('~port', 9090)
    address = rospy.get_param('~address', "0.0.0.0")

    external_port = rospy.get_param('~websocket_external_port', None)
    if external_port:
        try:
            external_port = int(external_port)
        except ValueError:
            external_port = None

    RosbridgeWebSocket.client_manager = ClientManager()

    # Get the glob strings and parse them as arrays.
    RosbridgeWebSocket.topics_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~topics_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0]
    RosbridgeWebSocket.services_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~services_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0]
    RosbridgeWebSocket.params_glob = [
        element.strip().strip("'")
        for element in rospy.get_param('~params_glob', '')[1:-1].split(',')
        if len(element.strip().strip("'")) > 0]

    if "--port" in sys.argv:
        idx = sys.argv.index("--port")+1
        if idx < len(sys.argv):
            port = int(sys.argv[idx])
        else:
            print("--port argument provided without a value.")
            sys.exit(-1)

    if "--address" in sys.argv:
        idx = sys.argv.index("--address")+1
        if idx < len(sys.argv):
            address = str(sys.argv[idx])
        else:
            print("--address argument provided without a value.")
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
            RosbridgeWebSocket.fragment_timeout = int(sys.argv[idx])
        else:
            print("--fragment_timeout argument provided without a value.")
            sys.exit(-1)

    if "--delay_between_messages" in sys.argv:
        idx = sys.argv.index("--delay_between_messages") + 1
        if idx < len(sys.argv):
            RosbridgeWebSocket.delay_between_messages = float(sys.argv[idx])
        else:
            print("--delay_between_messages argument provided without a value.")
            sys.exit(-1)

    if "--max_message_size" in sys.argv:
        idx = sys.argv.index("--max_message_size") + 1
        if idx < len(sys.argv):
            value = sys.argv[idx]
            if value == "None":
                RosbridgeWebSocket.max_message_size = None
            else:
                RosbridgeWebSocket.max_message_size = int(value)
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

    if "--topics_glob" in sys.argv:
        idx = sys.argv.index("--topics_glob") + 1
        if idx < len(sys.argv):
            value = sys.argv[idx]
            if value == "None":
                RosbridgeWebSocket.topics_glob = []
            else:
                RosbridgeWebSocket.topics_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
        else:
            print("--topics_glob argument provided without a value. (can be None or a list)")
            sys.exit(-1)

    if "--services_glob" in sys.argv:
        idx = sys.argv.index("--services_glob") + 1
        if idx < len(sys.argv):
            value = sys.argv[idx]
            if value == "None":
                RosbridgeWebSocket.services_glob = []
            else:
                RosbridgeWebSocket.services_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
        else:
            print("--services_glob argument provided without a value. (can be None or a list)")
            sys.exit(-1)

    if "--params_glob" in sys.argv:
        idx = sys.argv.index("--params_glob") + 1
        if idx < len(sys.argv):
            value = sys.argv[idx]
            if value == "None":
                RosbridgeWebSocket.params_glob = []
            else:
                RosbridgeWebSocket.params_glob = [element.strip().strip("'") for element in value[1:-1].split(',')]
        else:
            print("--params_glob argument provided without a value. (can be None or a list)")
            sys.exit(-1)

    if ("--bson_only_mode" in sys.argv) or bson_only_mode:
        RosbridgeWebSocket.bson_only_mode = bson_only_mode

    if "--websocket_ping_interval" in sys.argv:
        idx = sys.argv.index("--websocket_ping_interval") + 1
        if idx < len(sys.argv):
            ping_interval = float(sys.argv[idx])
        else:
            print("--websocket_ping_interval argument provided without a value.")
            sys.exit(-1)

    if "--websocket_ping_timeout" in sys.argv:
        idx = sys.argv.index("--websocket_ping_timeout") + 1
        if idx < len(sys.argv):
            ping_timeout = float(sys.argv[idx])
        else:
            print("--websocket_ping_timeout argument provided without a value.")
            sys.exit(-1)

    if "--websocket_external_port" in sys.argv:
        idx = sys.argv.index("--websocket_external_port") + 1
        if idx < len(sys.argv):
            external_port = int(sys.argv[idx])
        else:
            print("--websocket_external_port argument provided without a value.")
            sys.exit(-1)

    # To be able to access the list of topics and services, you must be able to access the rosapi services.
    if RosbridgeWebSocket.services_glob:
        RosbridgeWebSocket.services_glob.append("/rosapi/*")

    Subscribe.topics_glob = RosbridgeWebSocket.topics_glob
    Advertise.topics_glob = RosbridgeWebSocket.topics_glob
    Publish.topics_glob = RosbridgeWebSocket.topics_glob
    AdvertiseService.services_glob = RosbridgeWebSocket.services_glob
    UnadvertiseService.services_glob = RosbridgeWebSocket.services_glob
    CallService.services_glob = RosbridgeWebSocket.services_glob

    # Support the legacy "" address value.
    # The socket library would interpret this as INADDR_ANY.
    if not address:
        address = '0.0.0.0'

    ##################################################
    # Done with parameter handling                   #
    ##################################################

    def handle_compression_offers(offers):
        if not use_compression:
            return
        for offer in offers:
            if isinstance(offer, PerMessageDeflateOffer):
                return PerMessageDeflateOfferAccept(offer)

    if certfile is not None and keyfile is not None:
        protocol = 'wss'
        context_factory = ssl.DefaultOpenSSLContextFactory(keyfile, certfile)
    else:
        protocol = 'ws'
        context_factory = None

    # For testing purposes, use an ephemeral port if port == 0.
    if port == 0:
        rospy.loginfo('Rosbridge WebSocket Picking an ephemeral port')
        port = get_ephemeral_port()
    # Write the actual port as a param for tests to read.
    rospy.set_param('~actual_port', port)

    uri = '{}://{}:{}'.format(protocol, address, port)
    factory = WebSocketServerFactory(uri, externalPort=external_port)
    factory.protocol = RosbridgeWebSocket
    # https://github.com/crossbario/autobahn-python/commit/2ef13a6804054de74eb36455b58a64a3c701f889
    if LooseVersion(autobahn.__version__) < LooseVersion("0.15.0"):
        factory.setProtocolOptions(
            perMessageCompressionAccept=handle_compression_offers,
            autoPingInterval=ping_interval,
            autoPingTimeout=ping_timeout,
        )
    else:
        factory.setProtocolOptions(
            perMessageCompressionAccept=handle_compression_offers,
            autoPingInterval=ping_interval,
            autoPingTimeout=ping_timeout,
            allowNullOrigin=null_origin,
        )

    connected = False
    while not connected and not rospy.is_shutdown():
        try:
            listenWS(factory, context_factory)
            rospy.loginfo('Rosbridge WebSocket server started at {}'.format(uri))
            connected = True
        except CannotListenError as e:
            rospy.logwarn("Unable to start server: " + str(e) +
                          " Retrying in " + str(retry_startup_delay) + "s.")
            rospy.sleep(retry_startup_delay)

    rospy.on_shutdown(shutdown_hook)
    reactor.run()
