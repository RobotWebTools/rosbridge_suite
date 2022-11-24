from threading import Thread

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_action_class,
    get_action_goal_instance,
)
from unique_identifier_msgs.msg import UUID


class InvalidActionException(Exception):
    def __init__(self, actiontype):
        Exception.__init__(self, "Action %s does not exist" % actiontype)


class ActionClientHandle:
    def __init__(self, action_name, action_type, node_handle):
        self.action_name = action_name
        self.action_type = action_type
        self.node_handle = node_handle

        # raise exception if action type specified is None
        if self.action_type is None:
            raise InvalidActionException(action_type)

        # loads the class of action type
        action_class = get_action_class(action_type)
        self.action_client = ActionClient(node_handle, action_class, action_name)
        self.node_handle.get_logger().info(
            f" Created Action Client: {action_name} of type: {action_type}"
        )

        # create a client for cancel goal service call
        self.cancel_client = self.node_handle.create_client(
            CancelGoal, self.action_name + "/_action/cancel_goal"
        )

    def cancel_goal_call(self):
        """
        Sends a cancel goal service call.It cancels all active and pending goals
         of the action server by providing zeros to both goal id and stamp.
        """
        msg = CancelGoal.Request()
        msg.goal_info.goal_id = UUID(uuid=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        msg.goal_info.stamp.nanosec = 0
        msg.goal_info.stamp.sec = 0

        # create a call with the cancel request
        result = self.cancel_client.call(msg)
        if result is not None:
            # Turn the response into JSON and pass to the callback
            json_response = extract_values(result)
        else:
            raise Exception(result)

        return json_response

    def unregister(self):
        # cancel goals if any before destroy the client
        self.cancel_goal_call()
        self.cancel_client.destroy()
        self.action_client.destroy()


class GoalHandle(Thread):
    def __init__(
        self, action_client, goal_msg, success_callback, error_callback, feedback_callback
    ):
        """
        Create a goal handle for the specified action.
        Use start() to start in a separate thread or run() to run in this thread.

        Keyword Arguments:
        -----------------
        action_client -- the action client on which the goal is handled
        goal_msg -- arguments to pass to the action.  Can be an
        ordered list, or a dict of name-value pairs.  Anything else will be
        treated as though no arguments were provided (which is still valid for
        some kinds of action).
        success_callback -- a callback to call with the JSON result of the
        goal.
        error_callback -- a callback to call if an error occurs. The
        callback will be passed the exception that caused the failure of goal.
        feedback_callback -- a callback to call with the feedback while the goal is executing if opted.
        """
        Thread.__init__(self)
        self.daemon = True
        self.goal_msg = goal_msg
        self.success = success_callback
        self.error = error_callback
        self.feedback = feedback_callback
        self.client = action_client

    def run(self):
        try:
            # Call the action and pass the result to the success handler
            self.success(self.start_goal(self.goal_msg))
        except Exception as e:
            # On error, just pass the exception to the error handler
            self.error(e)

    def args_to_action_goal_instance(self, inst, args):
        """
        Populate a action goal instance with the provided args.
        args can be a dictionary of values, or a list, or None
        Propagates any exceptions that may be raised.
        """
        msg = {}
        if isinstance(args, dict):
            msg = args
        elif isinstance(args, list):
            msg = dict(zip(inst.get_fields_and_field_types().keys(), args))

        # Populate the provided instance, propagating any exceptions
        populate_instance(msg, inst)

    def start_goal(self, goal_msg):
        if not self.client.action_client.wait_for_server(timeout_sec=10.0):
            self.client.node_handle.get_logger().warning(
                f" Timeout: Action Server for Client: {self.client.action_name} not available. Goal is ignored "
            )
            raise Exception("Action Server Not Available")

        inst = get_action_goal_instance(self.client.action_type)
        # Populate the goal instance with the provided goal args
        self.args_to_action_goal_instance(inst, goal_msg)

        # send the goal and wait for the goal future to be accepted
        send_goal_future = self.client.action_client.send_goal_async(inst, self.feedback)
        rclpy.spin_until_future_complete(self.client.node_handle, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise Exception("Action Goal was rejected!")
        self.client.node_handle.get_logger().info(
            f"Goal is accepted by the action server: {self.client.action_name}."
        )

        # get the result
        result = goal_handle.get_result()

        # return the result of the goal if succeeded.
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Turn the response into JSON and pass to the callback
            json_response = extract_values(result.result)
        else:
            raise Exception(status)

        return json_response
