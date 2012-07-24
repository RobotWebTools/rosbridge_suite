from rosbridge_library.protocol import InvalidArgumentException
from rosbridge_library.protocol import MissingArgumentException


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
                fieldtype - the expected python type of the field

        Throws:
        MissingArgumentException -- if a field is mandatory but not present in
        the message
        InvalidArgumentException -- if a field is present but not of the type
        specified by fieldtype

        """
        for mandatory, fieldname, fieldtype in types_info:
            if mandatory and fieldname not in msg:
                raise MissingArgumentException("Expected a %s field but none was found." % fieldname)
            elif fieldname in msg and not isinstance(msg[fieldname], fieldtype):
                raise InvalidArgumentException("Expected field %s to be of type %s. Invalid value: %s" % (fieldname, fieldtype.__name__, msg[fieldname]))

