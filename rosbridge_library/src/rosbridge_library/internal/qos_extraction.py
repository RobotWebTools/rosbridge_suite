from typing import Any

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Duration

from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
)

DurabilityPolicies = [
    "system_default",
    "transient_local",
    "volatile",
    "unknown",
]

HistoryPolicies = [
    "system_default",
    "keep_last",
    "keep_all",
    "unknown",
]

LivelinessPolicies = [
    "system_default",
    "automatic",
    "",  # enum 2 is empty in rmw
    "manual_by_topic",
    "unknown",
]

ReliabilityPolicies = [
    "system_default",
    "reliable",
    "best_effort",
    "unknown",
]


def ExtractDuration(json_duration: list | str) -> Duration:
    if type(json_duration) is str:
        _ = json_duration.lower()
        if _ == "unspecified":
            return Duration(seconds=0, nanoseconds=0)
        if _ == "infinite":
            return Duration(seconds=9223372036, nanoseconds=854775807)
    elif type(json_duration) is list:
        if len(json_duration) == 2:
            return Duration(seconds=json_duration[0], nanoseconds=json_duration[1])
        if len(json_duration) == 1:
            return Duration(seconds=json_duration[0])
    return Duration(seconds=0, nanoseconds=0)


def ExtractQoSProfile(qosobj: dict[str, Any] | int | None) -> QoSProfile | None:
    qos: QoSProfile | int | None = None
    if type(qosobj) is int:
        qos = QoSProfile(depth=qosobj)
    elif type(qosobj) is dict:
        _ = qosobj.get("history")
        if type(_) is str:
            _ = _.lower()
            history = HistoryPolicies.index(_)
        else:
            history = _ if _ is not None else HistoryPolicy.SYSTEM_DEFAULT

        _ = qosobj.get("depth", 100)
        depth = _

        _ = qosobj.get("reliability")
        if type(_) is str:
            _ = _.lower()
            reliability = ReliabilityPolicies.index(_)
        else:
            reliability = _ if _ is not None else ReliabilityPolicy.SYSTEM_DEFAULT

        _ = qosobj.get("durability")
        if type(_) is str:
            _ = _.lower()
            durability = DurabilityPolicies.index(_)
        else:
            durability = _ if _ is not None else DurabilityPolicy.SYSTEM_DEFAULT

        _ = qosobj.get("deadline", [])
        deadline = ExtractDuration(_)

        _ = qosobj.get("lifespan", [])
        lifespan = ExtractDuration(_)

        _ = qosobj.get("liveliness")
        if type(_) is str:
            _ = _.lower()
            liveliness = LivelinessPolicies.index(_)
        else:
            liveliness = _ if _ is not None else LivelinessPolicy.SYSTEM_DEFAULT

        _ = qosobj.get("liveliness_lease_duration", [])
        liveliness_lease_duration = ExtractDuration(_)

        _ = qosobj.get("avoid_ros_namespace_conventions", False)
        avoid_ros_namespace_conventions = _

        qos = QoSProfile(
            history=history,
            depth=depth,
            reliability=reliability,
            durability=durability,
            deadline=deadline,
            lifespan=lifespan,
            liveliness=liveliness,
            liveliness_lease_duration=liveliness_lease_duration,
            avoid_ros_namespace_conventions=avoid_ros_namespace_conventions,
        )
    elif type(qosobj) is not None:
        raise InvalidArgumentException(qosobj)
    return qos
