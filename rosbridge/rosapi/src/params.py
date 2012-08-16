import rospy

from json import loads, dumps


""" Methods to interact with the param server.  Values have to be passed
as JSON in order to facilitate dynamically typed SRV messages """


def set_param(name, value):
    d = None
    try:
        d = loads(value)
    except ValueError:
        raise Exception("Due to the type flexibility of the ROS parameter server, the value argument to set_param must be a JSON-formatted string.")
    rospy.set_param(name, d)
    
    
def get_param(name, default):
    d = None
    if default is not "":
        try:
            d = loads(default)
        except ValueError:
            d = default
    return dumps(rospy.get_param(name, d))


def has_param(name):
    return rospy.has_param(name)


def delete_param(name):
    if has_param(name):
        rospy.delete_param(name)
        
        
def search_param(name):
    return rospy.search_param(name)

def get_param_names():
    return rospy.get_param_names()