from __future__ import annotations

import fnmatch
from typing import TYPE_CHECKING, NamedTuple

from rcl_interfaces.msg import ParameterType

if TYPE_CHECKING:
    from rclpy.node import Node


class Globs(NamedTuple):
    topics: list[str]
    services: list[str]
    params: list[str]


def get_globs(node: Node) -> Globs:
    def get_param(parameter_name: str) -> list[str]:
        parameter = node.get_parameter(parameter_name).get_parameter_value()

        parameter_value = ""
        if parameter.type == ParameterType.PARAMETER_STRING:
            parameter_value = parameter.string_value

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


def filter_globs(globs: list[str] | None, full_list: list[str]) -> list[str]:
    # If the globs are empty (weren't defined in the params), return the full list
    if globs is not None and len(globs) > 0:
        return list(filter(lambda x: any_match(x, globs), full_list))
    return full_list


def any_match(query: str, globs: list[str] | None) -> bool:
    return (
        globs is None or len(globs) == 0 or any(fnmatch.fnmatch(str(query), glob) for glob in globs)
    )
