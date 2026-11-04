from __future__ import annotations

import fnmatch
from typing import TYPE_CHECKING, NamedTuple

from rcl_interfaces.msg import ParameterType

if TYPE_CHECKING:
    from rclpy.node import Node


class Globs(NamedTuple):
    topics_pub: list[str] | None
    topics_sub: list[str] | None
    services: list[str] | None
    params: list[str] | None


def get_globs(node: Node) -> Globs:
    def get_param(parameter_name: str) -> list[str] | None:
        parameter = node.get_parameter(parameter_name).get_parameter_value()

        parameter_value = ""
        if parameter.type == ParameterType.PARAMETER_STRING:
            parameter_value = parameter.string_value

        if parameter_value == "":
            return None
        if parameter_value == "[]":
            return []

        # strips array delimiters in case of an array style value
        return [
            element.strip().strip("'")
            for element in parameter_value.strip("[").strip("]").split(",")
            if len(element.strip().strip("'")) > 0
        ]

    legacy_glob = get_param("topics_glob")
    pub_glob = get_param("topics_pub_glob")
    sub_glob = get_param("topics_sub_glob")

    # Append legacy topics glob into both pub and sub
    topics_pub_glob = legacy_glob if pub_glob is None else list(set(pub_glob + (legacy_glob or [])))
    topics_sub_glob = legacy_glob if sub_glob is None else list(set(sub_glob + (legacy_glob or [])))

    services_glob = get_param("services_glob")
    params_glob = get_param("params_glob")
    return Globs(topics_pub_glob, topics_sub_glob, services_glob, params_glob)


def filter_globs(globs: list[str] | None, full_list: list[str]) -> list[str]:
    # If no globs were defined in the params (""), do not filter and return
    # the full list. An empty list ("[]") still applies filtering and therefore
    # matches nothing, resulting in an empty list.
    if globs is not None:
        return list(filter(lambda x: any_match(x, globs), full_list))
    return full_list


def any_match(query: str, globs: list[str] | None) -> bool:
    if globs is None:
        return True
    return any(fnmatch.fnmatch(str(query), glob) for glob in globs)
