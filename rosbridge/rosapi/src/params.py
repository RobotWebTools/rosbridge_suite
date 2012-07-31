import rospy

from json import loads, dumps


""" Methods to interact with the param server.  Values have to be passed
as JSON in order to facilitate dynamically typed SRV messages """


def set_param(name, value):
    print "setting %s %s " % (name, value)
    rospy.set_param(name, loads(value))
    
    
def get_param(name, default):
    return dumps(rospy.get_param(name, loads(default)))


def has_param(name):
    return rospy.has_param(name)


def delete_param(name):
    if has_param(name):
        rospy.delete_param(name)
        
        
def search_param(name):
    return rospy.search_param(name)