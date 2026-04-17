from typing import Any, TypeVar

from rclpy.duration import Duration, Infinite
from rclpy.qos import (
    DeadlineBestAvailable,
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from rosbridge_library.internal.exceptions import (
    InvalidArgumentException,
)

HistoryPoliciesMapping = {
    "keep_last": HistoryPolicy.KEEP_LAST,
    "keep_all": HistoryPolicy.KEEP_ALL,
}

ReliabilityPoliciesMapping = {
    "reliable": ReliabilityPolicy.RELIABLE,
    "best_effort": ReliabilityPolicy.BEST_EFFORT,
    "best_available": ReliabilityPolicy.BEST_AVAILABLE,
}

DurabilityPoliciesMapping = {
    "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
    "volatile": DurabilityPolicy.VOLATILE,
    "best_available": DurabilityPolicy.BEST_AVAILABLE,
}


_PolicyT = TypeVar("_PolicyT")


def extract_enum_policy(policy_name: str, mapping: dict[str, _PolicyT]) -> _PolicyT:
    if not isinstance(policy_name, str):
        err_msg = f"Policy name must be a string, got {type(policy_name).__name__}"
        raise InvalidArgumentException(err_msg)
    policy_name = policy_name.lower()
    if policy_name in mapping:
        return mapping[policy_name]
    err_msg = f"'{policy_name}' is not a valid policy name. Valid options are: {', '.join(mapping.keys())}"
    raise InvalidArgumentException(err_msg)


def extract_duration(duration_raw: float | dict | str) -> Duration:
    if isinstance(duration_raw, (int, float)):
        return Duration(seconds=duration_raw)
    if isinstance(duration_raw, dict):
        secs = duration_raw.get("secs", 0)
        nsecs = duration_raw.get("nsecs", 0)
        return Duration(seconds=secs, nanoseconds=nsecs)
    if isinstance(duration_raw, str):
        if duration_raw.lower() == "infinite":
            return Infinite
        err_msg = f"'{duration_raw}' is not a valid duration string. Valid values are: 'infinite'"
        raise InvalidArgumentException(err_msg)
    err_msg = f"Duration must be a number, dict, or string, got {type(duration_raw).__name__}"
    raise InvalidArgumentException(err_msg)


def extract_qos_profile(qosobj: dict[str, Any]) -> QoSProfile:
    if not isinstance(qosobj, dict):
        err_msg = f"QoS profile must be a dict, got {type(qosobj).__name__}"
        raise InvalidArgumentException(err_msg)
    history: HistoryPolicy | None = None
    if "history" in qosobj:
        history = extract_enum_policy(qosobj["history"], HistoryPoliciesMapping)

    depth: int | None = None
    if "depth" in qosobj:
        depth = qosobj["depth"]
        if type(depth) is not int or depth < 0:
            err_msg = f"Depth must be a non-negative integer, got {depth}"
            raise InvalidArgumentException(err_msg)

    reliability: ReliabilityPolicy | None = None
    if "reliability" in qosobj:
        reliability = extract_enum_policy(qosobj["reliability"], ReliabilityPoliciesMapping)

    durability: DurabilityPolicy | None = None
    if "durability" in qosobj:
        durability = extract_enum_policy(qosobj["durability"], DurabilityPoliciesMapping)

    deadline: Duration | None = None
    if "deadline" in qosobj:
        deadline_raw = qosobj["deadline"]
        if isinstance(deadline_raw, str) and deadline_raw.lower() == "best_available":
            deadline = DeadlineBestAvailable
        else:
            deadline = extract_duration(deadline_raw)

    lifespan: Duration | None = None
    if "lifespan" in qosobj:
        lifespan = extract_duration(qosobj["lifespan"])

    return QoSProfile(
        history=history,
        depth=depth,
        reliability=reliability,
        durability=durability,
        deadline=deadline,
        lifespan=lifespan,
    )
