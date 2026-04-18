from typing import Any, TypeVar

from rclpy.duration import Duration, Infinite
from rclpy.qos import (
    DeadlineBestAvailable,
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
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
    if isinstance(duration_raw, bool):
        pass  # fall through to the type error below; bool must be checked before int/float
    elif isinstance(duration_raw, (int, float)):
        if duration_raw < 0:
            err_msg = f"Duration cannot be negative, got {duration_raw}"
            raise InvalidArgumentException(err_msg)
        return Duration(seconds=duration_raw)
    elif isinstance(duration_raw, dict):
        if "secs" not in duration_raw or "nsecs" not in duration_raw:
            err_msg = f"Duration dict must have 'secs' and 'nsecs' fields, got {duration_raw}"
            raise InvalidArgumentException(err_msg)
        secs = duration_raw["secs"]
        nsecs = duration_raw["nsecs"]
        if secs < 0 or nsecs < 0:
            err_msg = f"Duration cannot have negative values, got secs={secs}, nsecs={nsecs}"
            raise InvalidArgumentException(err_msg)
        return Duration(seconds=secs, nanoseconds=nsecs)
    elif isinstance(duration_raw, str):
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

    qos = QoSProfile(
        **qos_profile_system_default.get_c_qos_profile().to_dict()
    )  # Start with system default settings

    if "history" in qosobj:
        qos.history = extract_enum_policy(qosobj["history"], HistoryPoliciesMapping)

    if "depth" in qosobj:
        depth = qosobj["depth"]
        if type(depth) is not int or depth < 0:
            err_msg = f"Depth must be a non-negative integer, got {depth}"
            raise InvalidArgumentException(err_msg)
        qos.depth = depth

    if "reliability" in qosobj:
        qos.reliability = extract_enum_policy(qosobj["reliability"], ReliabilityPoliciesMapping)

    if "durability" in qosobj:
        qos.durability = extract_enum_policy(qosobj["durability"], DurabilityPoliciesMapping)

    if "deadline" in qosobj:
        deadline_raw = qosobj["deadline"]
        if isinstance(deadline_raw, str) and deadline_raw.lower() == "best_available":
            qos.deadline = DeadlineBestAvailable
        else:
            qos.deadline = extract_duration(deadline_raw)

    if "lifespan" in qosobj:
        qos.lifespan = extract_duration(qosobj["lifespan"])

    return qos
