from typing import Any

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
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


def ExtractQoSProfile(qosobj: dict[str, Any] | None) -> QoSProfile | None:
    qos: QoSProfile | None = None
    if qosobj is not None:
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

        _ = qosobj.get("deadline")
        deadline = _

        _ = qosobj.get("lifespan")
        lifespan = _

        _ = qosobj.get("liveliness")
        if type(_) is str:
            _ = _.lower()
            liveliness = LivelinessPolicies.index(_)
        else:
            liveliness = _ if _ is not None else LivelinessPolicy.SYSTEM_DEFAULT

        _ = qosobj.get("liveliness_lease_duration")
        liveliness_lease_duration = _

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
    return qos
