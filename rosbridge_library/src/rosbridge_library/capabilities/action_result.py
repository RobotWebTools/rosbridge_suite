from rosbridge_library.capability import Capability
from rosbridge_library.internal import message_conversion, ros_loader


class ActionResult(Capability):

    action_result_msg_fields = [
        (True, "action", str),
        (False, "id", str),
        (False, "values", dict),
        (True, "result", bool),
    ]

    def __init__(self, protocol):
        # Call superclass constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("action_result", self.action_result)

    def action_result(self, message):
        # Typecheck the args
        self.basic_type_check(message, self.action_result_msg_fields)

        # check for the action
        action_name = message["action"]
        if action_name in self.protocol.external_action_list:
            action_handler = self.protocol.external_action_list[action_name]
            # parse the message
            goal_id = message["id"]
            values = message["values"]
            # create a message instance
            result = ros_loader.get_action_result_instance(action_handler.action_type)
            message_conversion.populate_instance(values, result)
            # pass along the result
            action_handler.handle_result(goal_id, result)
        else:
            self.protocol.log(
                "error",
                f"Action {action_name} has not been advertised via rosbridge.",
            )
