from threading import Thread
from rospy import ServiceProxy
from rosservice import get_service_type
from rosbridge_library.internal.ros_loader import get_service_class
from rosbridge_library.internal.ros_loader import get_service_request_instance
from rosbridge_library.internal.message_conversion import populate_instance
from rosbridge_library.internal.message_conversion import extract_values


class InvalidServiceException(Exception):
    def __init__(self, servicename):
        Exception.__init__(self, "Service %s does not exist" % servicename)


class ServiceCaller(Thread):

    def __init__(self, service, args, success_callback, error_callback):
        """ Create a service caller for the specified service.  Use start()
        to start in a separate thread or run() to run in this thread.

        Keyword arguments:
        service          -- the name of the service to call
        args             -- arguments to pass to the service.  Can be an
        ordered list, or a dict of name-value pairs.  Anything else will be
        treated as though no arguments were provided (which is still valid for
        some kinds of service)
        success_callback -- a callback to call with the JSON result of the
        service call
        error_callback   -- a callback to call if an error occurs.  The
        callback will be passed the exception that caused the failure

         """
        Thread.__init__(self)
        self.daemon = True
        self.service = service
        self.args = args
        self.success = success_callback
        self.error = error_callback

    def run(self):
        try:
            # Call the service and pass the result to the success handler
            self.success(call_service(self.service, self.args))
        except Exception as e:
            # On error, just pass the exception to the error handler
            self.error(e)


def args_to_service_request_instance(service, inst, args):
    """ Populate a service request instance with the provided args

    args can be a dictionary of values, or a list, or None

    Propagates any exceptions that may be raised. """
    msg = {}
    if isinstance(args, list):
        msg = dict(zip(inst.__slots__, args))
    elif isinstance(args, dict):
        msg = args

    # Populate the provided instance, propagating any exceptions
    populate_instance(msg, inst)


def call_service(service, args=None):
    # Given the service name, fetch the type and class of the service,
    # and a request instance
    service_type = get_service_type(service)
    if service_type is None:
        raise InvalidServiceException(service)
    service_class = get_service_class(service_type)
    inst = get_service_request_instance(service_type)

    # Populate the instance with the provided args
    args_to_service_request_instance(service, inst, args)

    # Call the service
    proxy = ServiceProxy(service, service_class)
    response = proxy.call(inst)

    # Turn the response into JSON and pass to the callback
    json_response = extract_values(response)

    return json_response
