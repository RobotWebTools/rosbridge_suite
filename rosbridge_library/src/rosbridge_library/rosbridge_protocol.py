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

from rosbridge_library.capabilities.action_feedback import ActionFeedback
from rosbridge_library.capabilities.action_result import ActionResult
from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.advertise_action import AdvertiseAction
from rosbridge_library.capabilities.advertise_service import AdvertiseService
from rosbridge_library.capabilities.call_service import CallService
from rosbridge_library.capabilities.defragmentation import Defragment
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.send_action_goal import SendActionGoal
from rosbridge_library.capabilities.service_response import ServiceResponse
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.capabilities.unadvertise_action import UnadvertiseAction
from rosbridge_library.capabilities.unadvertise_service import UnadvertiseService
from rosbridge_library.protocol import Protocol


class RosbridgeProtocol(Protocol):
    """Adds the handlers for the rosbridge opcodes"""

    rosbridge_capabilities = [
        Advertise,
        Publish,
        Subscribe,
        Defragment,
        AdvertiseService,
        CallService,
        ServiceResponse,
        UnadvertiseService,
        AdvertiseAction,
        ActionFeedback,
        ActionResult,
        SendActionGoal,
        UnadvertiseAction,
    ]

    print("registered capabilities (classes):")
    for cap in rosbridge_capabilities:
        print(" -", str(cap))

    parameters = None

    def __init__(self, client_id, node_handle, parameters=None):
        self.parameters = parameters
        Protocol.__init__(self, client_id, node_handle)
        for capability_class in self.rosbridge_capabilities:
            self.add_capability(capability_class)
