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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
    MissingArgumentException,
)

if TYPE_CHECKING:
    from collections.abc import Sequence

    from rosbridge_library.protocol import Protocol


class Capability:
    """
    Handles the operation-specific logic of a rosbridge message.

    May define one or more opcodes to handle, for example 'publish' or
    'call_service'

    Each connected client receives its own capability instance, which are
    managed by the client's own protocol instance.

    Protocol.send() is available to send messages back to the client.
    """

    def __init__(self, protocol: Protocol) -> None:
        """
        Abstract class constructor.

        All capabilities require a handle to the containing protocol.

        :param protocol: The protocol instance for this capability instance
        """
        self.protocol = protocol

    def handle_message(self, message: dict[str, Any]) -> None:
        """
        Handle an incoming message.

        Called by the protocol after having already checked the message op code.

        :param message: The incoming message, deserialized into a dictionary
        """

    def finish(self) -> None:
        """
        Notify this capability that the client is finished.

        Tells the capability that it's time to free up resources.
        """

    def basic_type_check(
        self, msg: dict[str, Any], types_info: Sequence[tuple[bool, str, type | tuple[type, ...]]]
    ) -> None:
        """
        Perform basic typechecking on fields in msg.

        :param msg: A message, deserialized into a dictionary
        :param types_info: A sequence of tuples (mandatory, fieldname, fieldtype) where

            - mandatory - boolean, is the field mandatory
            - fieldname - the name of the field in the message
            - fieldtypes - the expected python type of the field or tuple of types

        :raises MissingArgumentException: If a field is mandatory but not present in the message
        :raises InvalidArgumentException: If a field is present but not of the type specified by
            fieldtype
        """
        for mandatory, fieldname, fieldtypes in types_info:
            if mandatory and fieldname not in msg:
                err_msg = f"Expected a {fieldname} field but none was found."
                raise MissingArgumentException(err_msg)
            if fieldname in msg:
                current_fieldtypes = fieldtypes
                if not isinstance(current_fieldtypes, tuple):
                    current_fieldtypes = (current_fieldtypes,)
                valid = False
                for typ in current_fieldtypes:
                    if isinstance(msg[fieldname], typ):
                        valid = True
                if not valid:
                    err_msg = (
                        f"Expected field {fieldname} to be one of {current_fieldtypes}. "
                        f"Invalid value: {msg[fieldname]}"
                    )
                    raise InvalidArgumentException(err_msg)
