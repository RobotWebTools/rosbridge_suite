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

from rosbridge_library.internal.exceptions import InvalidArgumentException
from rosbridge_library.internal.exceptions import MissingArgumentException


class Capability:
    """ Handles the operation-specific logic of a rosbridge message

    May define one or more opcodes to handle, for example 'publish' or
    'call_service'

    Each connected client receives its own capability instance, which are
    managed by the client's own protocol instance.

    Protocol.send() is available to send messages back to the client.

    """

    def __init__(self, protocol):
        """ Abstract class constructor.  All capabilities require a handle to
        the containing protocol.

        Keyword arguments:
        protocol -- the protocol instance for this capability instance

        """
        self.protocol = protocol

    def handle_message(self, message):
        """ Handle an incoming message.

        Called by the protocol after having already checked the message op code

        Keyword arguments:
        message -- the incoming message, deserialized into a dictionary

        """
        pass

    def finish(self):
        """ Notify this capability that the client is finished and that it's
        time to free up resources. """
        pass

    def basic_type_check(self, msg, types_info):
        """ Performs basic typechecking on fields in msg.

        Keyword arguments:
        msg        -- a message, deserialized into a dictoinary
        types_info -- a list of tuples (mandatory, fieldname, fieldtype) where
                mandatory - boolean, is the field mandatory
                fieldname - the name of the field in the message
                fieldtypes - the expected python type of the field or list of types

        Throws:
        MissingArgumentException -- if a field is mandatory but not present in
        the message
        InvalidArgumentException -- if a field is present but not of the type
        specified by fieldtype

        """
        for mandatory, fieldname, fieldtypes in types_info:
            if mandatory and fieldname not in msg:
                raise MissingArgumentException("Expected a %s field but none was found." % fieldname)
            elif fieldname in msg:
                if not isinstance(fieldtypes, tuple):
                    fieldtypes = (fieldtypes, )
                valid = False
                for typ in fieldtypes:
                    if isinstance(msg[fieldname], typ):
                        valid = True
                if not valid:
                    raise InvalidArgumentException("Expected field %s to be one of %s. Invalid value: %s" % (fieldname, fieldtypes, msg[fieldname]))

