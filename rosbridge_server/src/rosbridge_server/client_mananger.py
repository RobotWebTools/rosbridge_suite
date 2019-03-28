#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Intermodalics
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
#  * Neither the name of Intermodalics nor the names of its
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
import rospy
import threading
from rosbridge_msgs.msg import ConnectedClient, ConnectedClients
from std_msgs.msg import Int32


class ClientManager:
    def __init__(self):
        # Publisher for number of connected clients
        self._client_count_pub = rospy.Publisher(
          'client_count', Int32, queue_size=10, latch=True)
        # Publisher for connected clients
        self._conn_clients_pub = rospy.Publisher(
          'connected_clients', ConnectedClients, queue_size=10, latch=True)
        self._lock = threading.Lock()
        self._client_count = 0
        self._clients = {}
        self.__publish()

    def __publish(self):
        msg = ConnectedClients()
        msg.clients = list(self._clients.values())
        self._conn_clients_pub.publish(msg)
        self._client_count_pub.publish(len(msg.clients))

    def add_client(self, client_id, ip_address):
        with self._lock:
            client = ConnectedClient()
            client.ip_address = ip_address
            client.connection_time = rospy.Time.now()
            self._clients[client_id] = client
            self.__publish()

    def remove_client(self, client_id, ip_address):
        with self._lock:
            self._clients.pop(client_id, None)
            self.__publish()
