# Software License Agreement (BSD License)
#
# Copyright (c) 2023, PickNik Inc.
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
#  * Neither the name of the copyright holder nor the names of its
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

import fnmatch

from rosbridge_library.capability import Capability
from rosbridge_library.protocol import Protocol


class UnadvertiseAction(Capability):

    actions_glob = None

    def __init__(self, protocol: Protocol) -> None:
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("unadvertise_action", self.unadvertise_action)

    def unadvertise_action(self, message: dict) -> None:
        # parse the message
        action_name = message["action"]

        if UnadvertiseAction.actions_glob is not None and UnadvertiseAction.actions_glob:
            self.protocol.log(
                "debug",
                f"Action security glob enabled, checking action: {action_name}",
            )
            match = False
            for glob in UnadvertiseAction.actions_glob:
                if fnmatch.fnmatch(action_name, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", continuing action unadvertisement...",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    f"No match found for action, cancelling action unadvertisement for:{action_name}",
                )
                return
        else:
            self.protocol.log(
                "debug",
                "No action security glob, not checking action unadvertisement...",
            )

        # unregister action in ROS
        if action_name in self.protocol.external_action_list.keys():
            self.protocol.external_action_list[action_name].graceful_shutdown()
            del self.protocol.external_action_list[action_name]
            self.protocol.log("info", f"Unadvertised action {action_name}")
        else:
            self.protocol.log(
                "error",
                f"Action {action_name} has not been advertised via rosbridge, can't unadvertise.",
            )
