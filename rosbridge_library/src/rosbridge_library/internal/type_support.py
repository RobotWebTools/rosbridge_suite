# Software License Agreement (BSD License)
#
# Copyright (c) 2025, Fictionlab sp. z o.o.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations

from typing import Any, Protocol, TypeVar, runtime_checkable

from rclpy.action.client import ActionClient as _ActionClient
from rclpy.action.client import ClientGoalHandle as _ClientGoalHandle
from rclpy.action.server import ActionServer as _ActionServer
from rclpy.action.server import ServerGoalHandle as _ServerGoalHandle

try:
    from rosidl_pycommon.interface_base_classes import (
        BaseAction,
        BaseMessage,
        BaseService,
    )

    ROSMessage = BaseMessage
    ROSService = BaseService
    ROSAction = BaseAction

except ImportError:
    # Fallback to Protocols if interface base classes are not available
    # TODO: Remove this fallback once we drop support for Kilted

    @runtime_checkable
    class ROSMessage(Protocol):  # type: ignore[no-redef]
        """Protocol for ROS message types."""

        __slots__: list[str]
        _fields_and_field_types: dict[str, str]

        def get_fields_and_field_types(self) -> dict[str, str]:
            """Return a dictionary of field names to field types."""

    @runtime_checkable
    class ROSService(Protocol):  # type: ignore[no-redef]
        """Protocol for ROS service types."""

        Request: type[ROSMessage]
        Response: type[ROSMessage]
        Event: type[ROSMessage]

    @runtime_checkable
    class ROSAction(Protocol):  # type: ignore[no-redef]
        """Protocol for ROS action types."""

        Goal: type[ROSMessage]
        Result: type[ROSMessage]
        Feedback: type[ROSMessage]
        Impl: type[Any]


# Type variables for ROS types
ROSMessageT = TypeVar("ROSMessageT", bound=ROSMessage)
ROSServiceT = TypeVar("ROSServiceT", bound=ROSService)
ROSServiceRequestT = TypeVar("ROSServiceRequestT", bound=ROSMessage)
ROSServiceResponseT = TypeVar("ROSServiceResponseT", bound=ROSMessage)
ROSActionT = TypeVar("ROSActionT", bound=ROSAction)
ROSActionGoalT = TypeVar("ROSActionGoalT", bound=ROSMessage)
ROSActionResultT = TypeVar("ROSActionResultT", bound=ROSMessage)
ROSActionFeedbackT = TypeVar("ROSActionFeedbackT", bound=ROSMessage)

try:
    from rosidl_pycommon.interface_base_classes import BaseImpl

    ROSActionImplT = TypeVar("ROSActionImplT", bound=BaseImpl[Any, Any, Any])

    ActionClientType = _ActionClient[
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT
    ]
    ClientGoalHandleType = _ClientGoalHandle[
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT
    ]
    ActionServerType = _ActionServer[
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT
    ]
    ServerGoalHandleType = _ServerGoalHandle[
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT, ROSActionImplT
    ]

except ImportError:
    # Fallback to old type variables if BaseImpl is not available
    # TODO: Remove this fallback once we drop support for Kilted

    ROSActionImplT = TypeVar("ROSActionImplT")  # type: ignore[misc]

    ActionClientType = _ActionClient[  # type: ignore[misc]
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT
    ]
    ClientGoalHandleType = _ClientGoalHandle[  # type: ignore[misc]
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT
    ]
    ActionServerType = _ActionServer[  # type: ignore[misc]
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT
    ]
    ServerGoalHandleType = _ServerGoalHandle[  # type: ignore[misc]
        ROSActionGoalT, ROSActionResultT, ROSActionFeedbackT
    ]
