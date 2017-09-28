
import rospy

topics_glob = []
services_glob = []
params_glob = []


def get_globs():

    global topics_glob
    global services_glob
    global params_glob

    def get_param(param_name):
        param = rospy.get_param(param_name, '')
        # strips array delimiters in case of an array style value
        return [
            element.strip().strip("'")
            for element in param.strip('[').strip(']').split(',')
            if len(element.strip().strip("'")) > 0]

    topics_glob = get_param('~topics_glob')
    services_glob = get_param('~services_glob')
    params_glob = get_param('~params_glob')
