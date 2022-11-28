import fnmatch
from functools import partial

from rosbridge_library.capability import Capability
from rosbridge_library.internal.actions import ActionClientHandle, GoalHandle
from rosbridge_library.internal.message_conversion import extract_values


class ActionClientRequests(Capability):

    send_goal_msg_fields = [
        (True, "action_name", str),
        (True, "action_type", str),
        (False, "feedback", bool),
    ]

    destroy_client_msg_fields = [(True, "action_name", str)]

    cancel_goal_msg_fields = [(True, "action_name", str)]

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
            self.protocol.log(
                "debug", "Action security glob enabled, checking action clients: " + action_name
            )
            match = False
            for glob in ActionClientRequests.actions_glob:
                if fnmatch.fnmatch(action_name, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", creating Action client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action, cancelling creation of action client: "
                    + action_name,
                )
                return
        else:
            self.protocol.log("debug", "No action security glob, not checking Action client.")

        client_id = msg.get("id", None)
        s_cb = partial(self._success, client_id, action_name)
        e_cb = partial(self._failure, client_id, action_name)
        if feedback:
            f_cb = partial(self._feedback, client_id, action_name)
        else:
            f_cb = None

        if action_name not in self._actionclients:
            self._actionclients[action_name] = ActionClientHandle(
                action_name, action_type, self.protocol.node_handle
            )

        GoalHandle(self._actionclients[action_name], goal_msg, s_cb, e_cb, f_cb).start()

    def _success(self, cid, action_name, message):
        outgoing_message = {
            "op": "action_response",
            "response_type": "result",
            "name": action_name,
            "values": message,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _failure(self, cid, action_name, exc):
        outgoing_message = {
            "op": "action_response",
            "response_type": "error",
            "name": action_name,
            "values": str(exc),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def _feedback(self, cid, action_name, message):
        outgoing_message = {
            "op": "action_response",
            "response_type": "feedback",
            "name": action_name,
            "values": extract_values(message),
        }
        if cid is not None:
            outgoing_message["id"] = cid
        # TODO: fragmentation, compression
        self.protocol.send(outgoing_message)

    def destroy_client(self, msg):
        self.basic_type_check(msg, self.destroy_client_msg_fields)
        action_name = msg.get("action_name")

        if ActionClientRequests.actions_glob is not None and ActionClientRequests.actions_glob:
            self.protocol.log(
                "debug",
                "Action security glob enabled, checking action clients: " + action_name,
            )
            match = False
            for glob in ActionClientRequests.actions_glob:
                if fnmatch.fnmatch(action_name, glob):
                    self.protocol.log(
                        "debug",
                        "Found match with glob " + glob + ", killing client",
                    )
                    match = True
                    break
            if not match:
                self.protocol.log(
                    "warn",
                    "No match found for action client, cancelling destruction of action client: "
                    + action_name,
                )
                return
        else:
            self.protocol.log("debug", "No action security glob, not checking Action client.")

        if action_name not in self._actionclients:
            self.protocol.log("info", "action client %s not available" % action_name)
            return
        self._actionclients[action_name].unregister()
        del self._actionclients[action_name]

        if len(self._actionclients) == 0:
            self._actionclients.clear()

        self.protocol.log("info", "Destroyed Action Client %s" % action_name)

    def cancel_goal(self, msg):
        self.basic_type_check(msg, self.cancel_goal_msg_fields)
        action_name = msg.get("action_name")
        cid = msg.get("id", None)

        if action_name not in self._actionclients:
            self.protocol.log("info", "action client %s not available" % action_name)
            return

        result = self._actionclients[action_name].cancel_goal_call()

        outgoing_message = {
            "op": "action_response",
            "response_type": "cancel",
            "name": action_name,
            "values": result,
        }
        if cid is not None:
            outgoing_message["id"] = cid
        self.protocol.send(outgoing_message)

        self.protocol.log("info", "cancelled goals of %s" % action_name)

    def finish(self):
        for clients in self._actionclients.values():
            clients.unregister()
        self._actionclients.clear()
        self.protocol.unregister_operation("send_goal")
        self.protocol.unregister_operation("destroy_client")
        self.protocol.unregister_operation("cancel_goal")
