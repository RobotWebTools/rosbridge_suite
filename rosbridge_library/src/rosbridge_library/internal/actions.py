from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.expand_topic_name import expand_topic_name
from rclpy.action import ActionClient
from rosbridge_library.internal.message_conversion import (
    extract_values,
    populate_instance,
)
from rosbridge_library.internal.ros_loader import (
    get_action_class,
    get_action_goal_instance,
    get_action_result_instance,
    get_action_feedback_instance,
    get_action_status_instance
)

class InvalidActionException(Exception):
    def __init__(self, actionname):
        Exception.__init__(self, "Action %s does not exist" % actionname)


class ActionCaller(Thread):
    def __init__(self, action_type, action_name, args, success_callback, error_callback, node_handle):
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
        callback will be passed the exception that caused the failure
        node_handle      -- a ROS2 node handle to call actions.
        """
        Thread.__init__(self)
        self.daemon = True
        self.action_type = action_type
        self.action_name = action_name
        self.args = args
        self.success = success_callback
        self.error = error_callback
        self.node_handle = node_handle

    def run(self):
        try:
            # Call the action and pass the result to the success handler
            self.success(call_action(self.node_handle, self.action_type, self.action_name, self.args))
        except Exception as e:
            # On error, just pass the exception to the error handler
            self.error(e)




def args_to_action_goal_instance(inst, args):
    """Populate a action goal instance with the provided args

    args can be a dictionary of values, or a list, or None

    Propagates any exceptions that may be raised."""
    msg = {}
    print(f"dfdd : {args}")
    if isinstance(args, dict):
        msg = args    
    elif isinstance(args, list):
        msg = dict(zip(inst.get_fields_and_field_types().keys(), args))
   

    # Populate the provided instance, propagating any exceptions
    populate_instance(msg, inst)


def call_action(node_handle, action_type, action_name, args=None):
    # Given the action name, fetch the type and class of the action,
    # and a request instance

    # This should be equivalent to rospy.resolve_name.
    #action = expand_topic_name(action, node_handle.get_name(), node_handle.get_namespace())

    def _feedbackCallback(msg):
        
        #_feedback = msg.feedback
        node_handle.get_logger().info(f"feedback_msg: {msg.feedback}")
        return
    node_handle.get_logger().info(f" action server  {action_name} : {action_type} ")
   
    if action_type is None:
        raise InvalidActionException(action_type)
    # action_type is a tuple of types at this point; only one type is supported.
    #if len(action_type) > 1:
     #   node_handle.get_logger().warning(f"More than one action type detected: {action_type}")
    #action_type = action_type[0]

    action_class = get_action_class(action_type)
    inst = get_action_goal_instance(action_type)
    node_handle.get_logger().info(f"goal_instance {inst} ")
    # Populate the instance with the provided args
    args_to_action_goal_instance(inst, args)

    client = ActionClient(node_handle, action_class, action_name)
    node_handle.get_logger().info(f"Waiting for {action_name} action server")
    while not client.wait_for_server(timeout_sec=1.0):
        node_handle.get_logger().info(f" {action_name} action server not available, waiting...")


    send_goal_future = client.send_goal_async(inst, _feedbackCallback)
    rclpy.spin_until_future_complete(node_handle, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        raise  Exception('Action Goal was rejected!')
    
    

    result_future = goal_handle.get_result_async()
    goal_status = get_action_status_instance()
    node_handle.get_logger().info(f"goal status {goal_status} .")
    while result_future:
        rclpy.spin_until_future_complete(node_handle, result_future, timeout_sec=0.1)
        node_handle.get_logger().info(f" executing {result_future} ")
        if result_future.result():
            break
    
    status = result_future.result().status
    node_handle.get_logger().info(f"Action {action_name} status =  {status}")    
    node_handle.get_logger().info(f" result : = {result_future.result()} ")

    if status == goal_status.STATUS_SUCCEEDED:
        # Turn the response into JSON and pass to the callback
        json_response = extract_values(result_future.result().result)
    else:
        raise Exception(status)

    return json_response


rclpy.init()
node_ = Node("test_node")
json_ret = call_action(node_, 'action_tutorials_interfaces/action/Fibonacci', '/fibonacci', args= {"order": 17})
       
