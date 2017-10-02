#!/usr/bin/env python

import rospy
import fnmatch

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


def filter_globs(globs, full_list):
    # If the globs are empty (weren't defined in the params), return the full list
    if globs is not None and len(globs) > 0:
        return filter(lambda x: any_match(x, globs), full_list)
    else:
        return full_list


def any_match(query, globs):
    return globs is None or len(globs) == 0 or any(fnmatch.fnmatch(str(query), glob) for glob in globs)

