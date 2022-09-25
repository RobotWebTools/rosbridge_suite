from threading import Thread
import rclpy
from rclpy.action import ActionClient
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_action_class,
    get_action_goal_instance,
    get_action_status_instance
)

class InvalidActionException(Exception):
    def __init__(self, actionname):
        Exception.__init__(self, "Action %s does not exist" % actionname)


class ActionCaller(Thread):
    def __init__(self, action_type, action_name, args, success_callback, error_callback, feedback_callback, node_handle):
        """Create a action caller for the specified action.  Use start()
        to start in a separate thread or run() to run in this thread.

        Keyword arguments:
        action          -- the name of the action to call
        args             -- arguments to pass to the action.  Can be an
        ordered list, or a dict of name-value pairs.  Anything else will be
        treated as though no arguments were provided (which is still valid for
        some kinds of action)
        success_callback -- a callback to call with the JSON result of the
        action call
        error_callback   -- a callback to call if an error occurs.  The
        callback will be passed the exception that caused the failure.
        feedback_callback -- a callback to call with the feedback of the action.
        node_handle      -- a ROS2 node handle to call actions.
        """
        Thread.__init__(self)
        self.daemon = True
        self.action_type = action_type
        self.action_name = action_name
        self.args = args
        self.success = success_callback
        self.error = error_callback
        self.feedback = feedback_callback
        self.node_handle = node_handle

        if self.action_type is None:
            raise InvalidActionException(action_type)
        
        action_class = get_action_class(action_type)
        self.client = ActionClient(node_handle, action_class, action_name)
        self.node_handle.get_logger().info(f" Created Action Client: {self.action_name} of type: {self.action_type} successfully")

    def run(self):
        try:
            # Call the action and pass the result to the success handler
            self.success(self.start_action(self.args))
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


    def start_action(self,  args):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.node_handle.get_logger().info(f" Timeout: Action Server of Type: {self.action_type}  not available. Goal is ignored ")
            raise Exception("Action Server Not Available")
       
        inst = get_action_goal_instance(self.action_type)
        # Populate the instance with the provided args
        self.args_to_action_goal_instance(inst, args)

        send_goal_future = self.client.send_goal_async(inst, self.feedback)
        rclpy.spin_until_future_complete(self.node_handle, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            raise  Exception('Action Goal was rejected!')
        self.node_handle.get_logger().info(f"goal status == ACCEPTED .")
        result_future = goal_handle.get_result_async()
        goal_status = get_action_status_instance()
        while result_future:
            rclpy.spin_until_future_complete(self.node_handle, result_future, timeout_sec=0.1)
            if result_future.result():
                break
        
        status = result_future.result().status

        if status == goal_status.STATUS_SUCCEEDED:
            # Turn the response into JSON and pass to the callback
            json_response = extract_values(result_future.result().result)
        else:
            raise Exception(status)

        return json_response

    def unregister(self):
        #TODO cancel goal before destroy
            #self.client._cancel_goal_async()
            #self.client._goal_handles
        self.client.destroy()
        
