from threading import Thread
import rclpy
from rclpy.action import ActionClient
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_action_cancel_response_instance,
    get_action_class,
    get_action_goal_instance,
    get_action_status_instance
)


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
        self.node_handle.get_logger().info(f" Created Action Client: {action_name} of type: {action_type}")

    def call_off_goal(self):
        if len(self.action_client._goal_handles) == 0:
            self.node_handle.get_logger().info("no goal is active ")
            return None
        # get the goal uuid in the goal handle
        for goal_uuid in self.action_client._goal_handles:
            # send cancel request for goal_uuid's goal handle and get the response
            cancel_result = self.action_client._cancel_goal(self.action_client._goal_handles[goal_uuid]())
            self.node_handle.get_logger().info("gfdgdfgfdgfd ")
            cancel_status = get_action_cancel_response_instance
            if cancel_result.return_code == 0:
                self.node_handle.get_logger().info(f" Cancelled Goal : {cancel_result.goals_canceling}")
            return cancel_result

    def unregister(self):
        # cancel the pending goal if any before destroy the client
        self.call_off_goal()
        self.action_client.destroy()


class GoalHandle(Thread):
    def __init__(self, action_client, goal_msg, success_callback, error_callback, feedback_callback):
        """Create a action caller for the specified action.  Use start()
        to start in a separate thread or run() to run in this thread.

        Keyword arguments:
        action_client -- the action client on which the goal is handled
        goal_msg           -- arguments to pass to the action.  Can be an
        ordered list, or a dict of name-value pairs.  Anything else will be
        treated as though no arguments were provided (which is still valid for
        some kinds of action)
        success_callback -- a callback to call with the JSON result of the
        goal
        error_callback   -- a callback to call if an error occurs. The
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
        """Populate a action goal instance with the provided args

        args can be a dictionary of values, or a list, or None

        Propagates any exceptions that may be raised."""
        msg = {}
        if isinstance(args, dict):
            msg = args
        elif isinstance(args, list):
            msg = dict(zip(inst.get_fields_and_field_types().keys(), args))

        # Populate the provided instance, propagating any exceptions
        populate_instance(msg, inst)

    def start_goal(self,  goal_msg):
        if not self.client.action_client.wait_for_server(timeout_sec=10.0):
            self.client.node_handle.get_logger().info(f" Timeout: Action Server for Client Type: {self.client.action_type}  not available. Goal is ignored ")
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
        self.client.node_handle.get_logger().info("Goal is ACCEPTED by the action server.")

        # check the status of the goal handle untill it's done periodically
        result_future = goal_handle.get_result_async()
        goal_status = get_action_status_instance()
        while result_future:
            rclpy.spin_until_future_complete(self.client.node_handle, result_future, timeout_sec=0.1)
            if result_future.result():
                break
        
        # return the result of the goal if succeeded.
        status = result_future.result().status
        if status == goal_status.STATUS_SUCCEEDED:
            # Turn the response into JSON and pass to the callback
            json_response = extract_values(result_future.result().result)
        else:
            raise Exception(status)

        return json_response
