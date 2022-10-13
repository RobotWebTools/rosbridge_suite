import fnmatch
from functools import partial

from rosbridge_library.capability import Capability
from rosbridge_library.internal.actions import GoalHandle, ActionClientHandle
from rosbridge_library.internal.message_conversion import extract_values


class ActionClientRequests(Capability):

    send_goal_msg_fields = [
        (True, "action_name", str),
        (True, "action_type", str),
        (True, "feedback", bool),
        (False, "throttle_rate", int),
        (False, "fragment_size", int),
        (False, "queue_length", int),
        (False, "compression", str),
    ]

    destroy_client_msg_fields = [(True, "action_type", str)]

    cancel_goal_msg_fields = [(True, "action_type", str)]

    actions_glob = None

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("send_goal", self.send_goal)
        protocol.register_operation("destroy_client", self.destroy_client)
        protocol.register_operation("cancel_goal", self.cancel_goal)

        self._actionclients = {}

    def send_goal(self, msg):
        # Check the args
        self.basic_type_check(msg, self.send_goal_msg_fields)
        action_name = msg.get("action_name")
        action_type = msg.get("action_type")
        goal_msg = msg.get("goal_msg", [])
        feedback = msg.get("feedback", True)

        if ActionClientRequests.actions_glob is not None and ActionClientRequests.actions_glob:
            self.protocol.log("debug", "Action security glob enabled, checking action: " + action_type)
            match = False
            for glob in ActionClientRequests.actions_glob:
                if fnmatch.fnmatch(action_type, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", creating Action client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action, cancelling creation of action client type: " + action_type,
                )
                return
        else:
            self.protocol.log("debug", "No action security glob, not checking Action client.")

        client_id = msg.get("id", None)
        s_cb = partial(self._success, client_id, action_type)
        e_cb = partial(self._failure, client_id, action_type)
        if feedback:
            f_cb = partial(self._feedback, client_id, action_type)
        else:
            f_cb = None

        if action_type not in self._actionclients:
            self._actionclients[action_type] = ActionClientHandle(action_name, action_type, self.protocol.node_handle)

        GoalHandle(self._actionclients[action_type], goal_msg, s_cb, e_cb, f_cb).start()

    def _success(self, cid, action_type, message):
        outgoing_message = {
            "op": "action_response",
            "response_type": 'result',
            "type": action_type,
            "values": message,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid, action_type, exc):
        outgoing_message = {
            "op": "action_response",
            "response_type": "error",
            "type": action_type,
            "values": str(exc),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _feedback(self, cid, action_type, message):
        outgoing_message = {
            "op": "action_response",
            "response_type": 'feedback',
            "type": action_type,
            "values": extract_values(message),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def destroy_client(self, msg):
        self.basic_type_check(msg, self.destroy_client_msg_fields)
        action_type = msg.get("action_type")

        if ActionClientRequests.actions_glob is not None and ActionClientRequests.actions_glob:
            self.protocol.log("debug", "Action security glob enabled, checking action client of type:: " + action_type)
            match = False
            for glob in ActionClientRequests.actions_glob:
                if fnmatch.fnmatch(action_type, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", killing client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action client, cancelling destruction of action client of type: " + action_type,
                )
                return
        else:
            self.protocol.log("debug", "No action security glob, not checking Action client.")

        if action_type not in self._actionclients:
            return
        self._actionclients[action_type].unregister()
        del self._actionclients[action_type]

        if len(self._actionclients) == 0:
            self._actionclients.clear()

        self.protocol.log("info", "Destroyed Action Client of type %s" % action_type)

    def cancel_goal(self, msg):
        self.basic_type_check(msg, self.cancel_goal_msg_fields)
        action_type = msg.get("action_type")
        client_id = msg.get("id", None)

        if action_type not in self._actionclients:
            self.protocol.log("info", "action client of type %s not available" % action_type)
            return

        result = self._actionclients[action_type].call_off_goal()
        if result is not None:
            self._success(client_id, action_type, extract_values(result))
            self.protocol.log("info", "cancelled goal %s" % action_type)

    def finish(self):
        for clients in self._actionclients.values():
            clients.unregister()
        self._actionclients.clear()
        self.protocol.unregister_operation("send_goal")
        self.protocol.unregister_operation("destroy_client")
        self.protocol.unregister_operation("cancel_goal")
