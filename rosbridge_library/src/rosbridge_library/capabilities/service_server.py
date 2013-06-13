from rosbridge_library.capability import Capability

import rospy

class ServiceServer(Capability):
    opcode_advertise_service = "advertise_service"      # rosbridge-client -> rosbridge # register in protocol.py!
    opcode_unadvertise_service = "unadvertise_service"      # rosbridge-client -> rosbridge # register in protocol.py!
    opcode_service_response = "service_response"        # rosbridge-client -> rosbridge # register in protocol.py!

    client_opcode_service_request = "service_requests"         # rosbridge -> rosbridge-client # not to be registered in protocol.py!

    # have a dict that maps registered services to client id's
    #       service id
    #       client id
    #       service instance ( use a service template that just takes requests and passes them to clients by using methods below)
    #           ..this service instance has to track requests and responses by using a request id that is sent to rosbridge-client and used to identify which response has to be sent to which ros-client
    #           ..this service instance has to use a timeout during which it will wait for an incoming json service_response with correct id from
    service_list = None

    def __init__(self, protocol):
        Capability.__init__(self, protocol)

        protocol.register_operation(self.opcode_advertise_service, self.advertise_service)
        protocol.register_operation(self.opcode_service_response, self.service_response)

    def advertise_service(self):
        print "advertise_service called"
        # register service in ROS

        # have a superclass (or function - see below) that has 'callback' to client for service requests

    def unadvertise_service(self):
        print "unadvertise_service called"
        # register service in ROS

        # have a superclass (or function - see below) that has 'callback' to client for service requests

    def service_request(self):
        print "service_request called"
        # this be the callback for superclass that provides service to ros-side

        # pass service request to rosbridge_client with JSON opcode 'service_request'

    def service_response(self):
        print "service_response called"
        # this be the callback for rosbridge-client ; called from within ros-service

        # pass response to ros-side by sending through superclass that provides service to ros-side



# service_template could hold a list of "request-objects" that can be identified by request id..
#   these objects could just make the json call to rosbridge-side and wait (blocking) until they receive their answer through a service response
#     that they receive from ServiceServer  (each service request would spawn one of those objects)
#     -> send_request_to_rosbridge-service-provider():   just passes the json-call
#     -> send_response_to_ros-client():                  just returns the response-content /data as if the answer never left ros
# PROBLEM with idea above:
#    -> we register a function in ros whose return gets passed to ros-side
#       ..if not returning from that function we have no way to give back data to the request
# SOLUTION_1:
#    -> use the loop with checking response_list like described in code below
class ROS_Service_Template():
    service_request_timeout = None

    service_id = None
    service_name = None
    service_type = None

    request_list = None     # holds requests until they are answered (response successfully sent to ROS-client)  # probably not needed, but maybe good for retransmission of request or s.th. similar
    response_list = None    # holds service_responses until they are sent back to ROS-client


    def __init__(self):
        print "ROS_Service_Template used to create a rosbridge-ServiceInstance"
        self.spawn_ROS_service()

    def handle_service_request(self, req):
        print "handle_service_request called"
        # send JSON request to client that provides this service
        #    add request to request_list and pass id into JSON request
        # loop until timeout or response with request id is received (found in response_list)
        #    when response is found return content/data of response
        #       [return AddTwoIntsResponse(req.a + req.b)]


    def spawn_ROS_service(self):
        rospy.init_node(self.service_node_name)
        s = rospy.Service(self.service_name, self.service_type, self.handle_service_request)
        print "ROS service spawned.", self.service_id, self.service_type, self.service_id
        rospy.spin()


