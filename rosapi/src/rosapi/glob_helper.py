#!/usr/bin/env python

import fnmatch
from collections import namedtuple

from rcl_interfaces.msg import ParameterType

Globs = namedtuple("Globs", ["topics", "services", "params"])


def get_globs(node):
    def get_param(parameter_name):
        parameter_value = node.get_parameter(parameter_name).get_parameter_value()
        if parameter_value.type == ParameterType.PARAMETER_STRING:
            parameter_value = parameter_value.string_value
        else:
            parameter_value = ""
        # strips array delimiters in case of an array style value
        return [
            element.strip().strip("'")
            for element in parameter_value.strip("[").strip("]").split(",")
            if len(element.strip().strip("'")) > 0
        ]

    topics_glob = get_param("topics_glob")
    services_glob = get_param("services_glob")
    params_glob = get_param("params_glob")
    return Globs(topics_glob, services_glob, params_glob)


def filter_globs(globs, full_list):
    # If the globs are empty (weren't defined in the params), return the full list
    if globs is not None and len(globs) > 0:
        return list(filter(lambda x: any_match(x, globs), full_list))
    else:
        return full_list


def any_match(query, globs):
    return (
        globs is None or len(globs) == 0 or any(fnmatch.fnmatch(str(query), glob) for glob in globs)
    )
