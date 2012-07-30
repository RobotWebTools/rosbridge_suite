from rospy import logdebug, loginfo, logwarn, logerr
from json import dumps, loads


class InvalidArgumentException(Exception):
    pass


class MissingArgumentException(Exception):
    pass


class Protocol:
    """ The interface for a single client to interact with ROS.

    See rosbridge_protocol for the default protocol used by rosbridge

    The lifecycle for a Protocol instance is as follows:
    - Pass incoming messages from the client to incoming
    - Propagate outgoing messages to the client by overriding outgoing
    - Call finish to clean up resources when the client is finished

    """

    def __init__(self, client_id):
        """ Keyword arguments:
        client_id -- a unique ID for this client to take.  Uniqueness is
        important otherwise there will be conflicts between multiple clients
        with shared resources

        """
        self.client_id = client_id
        self.capabilities = []
        self.operations = {}

    def incoming(self, message_string):
        """ Process an incoming message from the client

        Keyword arguments:
        message_string -- the wire-level message sent by the client

        """
        msg = self.deserialize(message_string)
        if msg is None:
            return

        mid = None
        if "id" in msg:
            mid = msg["id"]

        if "op" not in msg:
            self.log("error", "Received a message without an op.  All messages require 'op' field with value one of: %s.  Original message was: %s" % (self.operations.keys(), message_string), mid)
            return

        op = msg["op"]
        if op not in self.operations:
            self.log("error", "Unknown operation: %s.  Allowed operations: %s" % (op, self.operations.keys()), mid)
            return

        try:
            self.operations[op](msg)
        except Exception as exc:
            self.log("error", "%s: %s" % (op, exc.message), mid)

    def outgoing(self, message):
        """ Pass an outgoing message to the client.  This method should be
        overridden.

        Keyword arguments:
        message -- the wire-level message to send to the client

        """
        print "outgoing message %s" % message
        pass

    def send(self, message, cid=None):
        """ Called internally in preparation for sending messages to the client

        This method pre-processes the message then passes it to the overridden
        outgoing method.

        Keyword arguments:
        message -- a dict of message values to be marshalled and sent
        cid     -- (optional) an associated id

        """
        serialized = self.serialize(message, cid)
        if serialized is not None:
            self.outgoing(serialized)

    def finish(self):
        """ Indicate that the client is finished and clean up resources.

        All clients should call this method after disconnecting.

        """
        for capability in self.capabilities:
            capability.finish()

    def serialize(self, msg, cid=None):
        """ Turns a dictionary of values into the appropriate wire-level
        representation.

        Default behaviour uses JSON.  Override to use a different container.

        Keyword arguments:
        msg -- the dictionary of values to serialize
        cid -- (optional) an ID associated with this.  Will be logged on err.

        Returns a JSON string representing the dictionary
        """
        try:
            return dumps(msg)
        except:
            if cid is not None:
                # Only bother sending the log message if there's an id
                self.log("error", "Unable to serialize %s message to client"
                         % msg["op"], cid)
            return None

    def deserialize(self, msg, cid=None):
        """ Turns the wire-level representation into a dictionary of values

        Default behaviour assumes JSON. Override to use a different container.

        Keyword arguments:
        msg -- the wire-level message to deserialize
        cid -- (optional) an ID associated with this.  Is logged on error

        Returns a dictionary of values

        """
        try:
            return loads(msg)
        except:
            self.log("error",
             "Unable to deserialize message from client: %s" % msg, cid)
            return None

    def register_operation(self, opcode, handler):
        """ Register a handler for an opcode

        Keyword arguments:
        opcode  -- the opcode to register this handler for
        handler -- a callback function to call for messages with this opcode

        """
        self.operations[opcode] = handler

    def unregister_operation(self, opcode):
        """ Unregister a handler for an opcode

        Keyword arguments:
        opcode -- the opcode to unregister the handler for

        """
        if opcode in self.operations:
            del self.operations[opcode]

    def add_capability(self, capability_class):
        """ Add a capability to the protocol.

        This method is for convenience; assumes the default capability
        constructor

        Keyword arguments:
        capability_class -- the class of the capability to add

        """
        self.capabilities.append(capability_class(self))

    def log(self, level, message, lid=None):
        """ Log a message to the client.  By default just sends to stdout

        Keyword arguments:
        level   -- the logger level of this message
        message -- the string message to send to the user
        lid     -- an associated for this log message

        """
        stdout_formatted_msg = None
        if lid is not None:
            stdout_formatted_msg = "[Client %s] [id: %s] %s" % (self.client_id, lid, message)
        else:
            stdout_formatted_msg = "[Client %s] %s" % (self.client_id, message)

        if level == "error" or level == "err":
            logerr(stdout_formatted_msg)
        elif level == "warning" or level == "warn":
            logwarn(stdout_formatted_msg)
        elif level == "info" or level == "information":
            loginfo(stdout_formatted_msg)
        else:
            logdebug(stdout_formatted_msg)
