# rosbridge v2.0.0 Protocol Specification <!-- omit in toc -->

This document defines the rosbridge protocol and its supported operations.
The protocol is built around structured message objects (e.g., JSON or CBOR) with an `op` field that identifies the operation being performed.
The protocol is transport-agnostic and can be carried over WebSockets, TCP, or other suitable transports.

This document also describes the intended direction of the rosbridge server implementation.
The default server implementation uses WebSockets and separates message parsing from the underlying transport so protocol operations remain easy to extend.

## Version history <!-- omit in toc -->

The protocol version is specified as a semantic version number in the format `MAJOR.MINOR.PATCH`, where:

- The major version number (the first number) is incremented for breaking changes, such as changing the encoding or format of an existing operation in a way that would cause existing clients to fail without modification.
- The minor version number (the second number) is incremented for non-breaking additions, such as adding a new operation or adding new optional fields to an existing operation.
- The patch version number (the third number) is incremented for non-breaking changes that don't add or modify any operation fields, such as changing the default QoS settings, modifying the behavior of existing fields or marking existing fields as deprecated.
- Editorial improvements, clarifications, and corrections to the document text do not increment the version; those changes are tracked in the repository history.

| Version | Summary |
|---------|---------|
| 2.0.0 | Initial versioned release. Covers topic, service, and action operations; Base64, PNG, CBOR and CBOR-RAW encodings; fragmentation; interface type notation; and default QoS settings for publishers and subscribers. |

## Table of Contents <!-- omit in toc -->

- [1. Message envelope](#1-message-envelope)
- [2. Operation summary](#2-operation-summary)
- [3. Data Encoding and Transformation](#3-data-encoding-and-transformation)
  - [3.1 Base64 encoding of byte arrays](#31-base64-encoding-of-byte-arrays)
  - [3.2 Fragmentation ( _fragment_ ) \[experimental\]](#32-fragmentation--fragment--experimental)
  - [3.3 PNG compression ( _png_ ) \[experimental\]](#33-png-compression--png--experimental)
  - [3.4 CBOR encoding ( _cbor_ )](#34-cbor-encoding--cbor-)
  - [3.5 CBOR-RAW encoding ( _cbor-raw_ )](#35-cbor-raw-encoding--cbor-raw-)
- [4. Operation specifications](#4-operation-specifications)
  - [4.1 Interface type notation](#41-interface-type-notation)
  - [4.2 Default QoS settings](#42-default-qos-settings)
  - [4.3 Topic operations](#43-topic-operations)
    - [4.3.1 advertise (C → S)](#431-advertise-c--s)
    - [4.3.2 unadvertise (C → S)](#432-unadvertise-c--s)
    - [4.3.3 publish (C ↔ S)](#433-publish-c--s)
    - [4.3.4 subscribe (C → S)](#434-subscribe-c--s)
    - [4.3.5 unsubscribe (C → S)](#435-unsubscribe-c--s)
  - [4.4 Service operations](#44-service-operations)
    - [4.4.1 advertise\_service (C → S)](#441-advertise_service-c--s)
    - [4.4.2 unadvertise\_service (C → S)](#442-unadvertise_service-c--s)
    - [4.4.3 call\_service (C ↔ S)](#443-call_service-c--s)
    - [4.4.4 service\_response (C ↔ S)](#444-service_response-c--s)
  - [4.5 Action operations](#45-action-operations)
    - [4.5.1 advertise\_action (C → S)](#451-advertise_action-c--s)
    - [4.5.2 unadvertise\_action (C → S)](#452-unadvertise_action-c--s)
    - [4.5.3 send\_action\_goal (C ↔ S)](#453-send_action_goal-c--s)
    - [4.5.4 cancel\_action\_goal (C ↔ S)](#454-cancel_action_goal-c--s)
    - [4.5.5 action\_feedback (C ↔ S)](#455-action_feedback-c--s)
    - [4.5.6 action\_result (C ↔ S)](#456-action_result-c--s)

## 1. Message envelope

A rosbridge message is, at minimum, a structured message object with a string field called `op`.
For example:

```json
{ "op": "Example" }
```

The `op` field identifies the operation being performed.
Messages with different values for `op` may be handled differently.

As long as the message is a valid object containing the `op` field, it is a valid rosbridge message.

Optionally, a message can also provide an arbitrary string ID:

```json
{
  "op": "Example",
  "id": "fred"
}
```

If an `id` is provided with a message to the server, then related response messages will typically contain that ID as well, allowing protocol messages on the wire to be correlated with the initiating request.
Server-side log messages may also include the `id` for correlation purposes; these logs are implementation-specific and are not delivered to clients as part of the rosbridge protocol itself.

Semantically, the `id` does not identify a single message.
Instead, it identifies an interaction, which may consist of multiple back-and-forth operations.

## 2. Operation summary

The rosbridge protocol defines a number of different operations.

Direction legend:

- **C → S**: client to server
- **S → C**: server to client
- **C ↔ S**: either direction

The `C ↔ S` operations are valid in either direction depending on which side has advertised the corresponding topic, service, or action interface.

Message compression / transformation:

- **fragment** – C ↔ S – part of a fragmented message
- **png** – S → C – a message compressed as a PNG image

Topic operations:

- **advertise** – C → S – advertise that the client will publish on a topic
- **unadvertise** – C → S – stop advertising that the client will publish on a topic
- **publish** – C ↔ S – publish a message on a topic
- **subscribe** – C → S – subscribe to a topic to receive updates
- **unsubscribe** – C → S – unsubscribe from a topic to stop receiving updates

Service operations:

- **advertise_service** – C → S – advertise an external service server
- **unadvertise_service** – C → S – stop advertising an external service server
- **call_service** – C ↔ S – invoke a service
- **service_response** – C ↔ S – return a service response

Action operations:

- **advertise_action** – C → S – advertise an external action server
- **unadvertise_action** – C → S – stop advertising an external action server
- **send_action_goal** – C ↔ S – send an action goal
- **cancel_action_goal** – C ↔ S – cancel an action goal
- **action_feedback** – C ↔ S – report action feedback
- **action_result** – C ↔ S – report an action result

In general, operation opcodes that initiate an action are verb-like, such as `subscribe`, `publish`, and `call_service`.
Feedback, result, and status-bearing messages often use noun or noun-phrase opcodes such as `service_response`, `action_feedback` and `action_result`.
These naming patterns are descriptive only and do not imply that a given opcode is sent exclusively by either the client or the server.

To keep the protocol simple and focused, the rosbridge protocol **does not** handle ROS parameter operations (getting, setting, listing parameters, etc.) or querying the ROS graph (listing nodes, topics, services, and their types).
Instead, it delegates such operations to dedicated ROS services, such as those provided by the [rosapi] package.

## 3. Data Encoding and Transformation

By default, rosbridge messages are encoded as JSON text.
The rosbridge protocol also provides alternative encodings and message transformations for cases where binary data, large payloads, or performance requirements make the default JSON encoding less suitable.

### 3.1 Base64 encoding of byte arrays

When the rosbridge server sends messages containing `uint8[]` or `char[]` fields, these byte arrays are encoded as base64 strings.
This reduces message size by up to 60% compared to sending the same data as a list of numbers when encoded in JSON.

For example, a message containing the following fields:

```
uint8[] data1 = [0, 0, 0, 0]
uint8[] data2 = [255, 255, 255, 255]
```

Will be transmitted as:

```json
{
  "data1": "AAAAAA==",
  "data2": "/////w=="
}
```

The string value is the base64-encoded representation of the byte array.
Byte arrays may be sent to the server as either a base64 string or a list of numbers, but they will be re-encoded as a base64 string before being sent to other clients.

### 3.2 Fragmentation ( _fragment_ ) [experimental]

Messages may be fragmented if they are particularly large, or if the client requests fragmentation.
A fragmented message has the following format:

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"fragment"` |
| `id` | required | string | Identifies which fragments belong to the same original message. |
| `data` | required | string | A chunk of the original message payload. Concatenating all chunks in order reconstructs the original serialized message. |
| `num` | required | integer | Zero-based index of this fragment within the sequence. |
| `total` | required | integer | Total number of fragments that make up the original message. |

To fragment a message, its serialized payload is taken and split up into multiple substrings or byte arrays.
For each chunk, a fragment message is constructed, with the data field of the fragment populated by the chunk.

To reconstruct an original message, the data fields of the fragments are concatenated, resulting in the serialized payload of the original message.

### 3.3 PNG compression ( _png_ ) [experimental]

Some messages (such as images and maps) can be extremely large, and for efficiency reasons we may wish to transfer them as PNG-encoded bytes.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"png"` |
| `data` | required | string | Base64-encoded PNG image whose pixel data encodes the serialized payload of the original message. |

To construct a PNG compressed message, the serialized payload of the original message is taken and interpreted as an RGB image.
The image is then saved as a PNG and the bytes are base64-encoded as a string.
This string is then used as the `data` field.

Currently, only Server to Client `png` messages are supported.
The server does not support receiving PNG-compressed messages from clients in the current version of the protocol.

### 3.4 CBOR encoding ( _cbor_ )

[CBOR] encoding is the fastest compression method for messages containing large blobs of data, such as byte arrays and numeric typed arrays.

When CBOR compression is requested by a subscriber, a binary message will be produced instead of a JSON string.
Once decoded, the message will contain a normal protocol message.

The implementation uses [draft typed array tags] for efficient packing of homogeneous arrays.
At the moment, only little-endian packing is supported.

### 3.5 CBOR-RAW encoding ( _cbor-raw_ )

While CBOR encodes the entire message as CBOR, sometimes it is desirable to get the raw binary message in the ROS 2 serialized message format.

This can be useful in several cases:

- Your application already knows how to parse raw ROS 2 message data or data stored in ROS 2 bag files,
  which means that you can use consistent code paths for both recorded and live messages.
- You want to parse messages as late as possible, or in parallel, e.g. only in the thread or WebWorker that cares about the message.
  Delaying the parsing of the message means that moving or copying the message to the thread is cheaper when it's in binary form, since no serialization between threads is necessary.
- You only care about part of the message, and don't need to parse the rest of it.
- You really care about performance; no conversion between the ROS 2 binary message format and CBOR is done in the rosbridge server.

The format is similar to CBOR above, but instead of the `msg` field containing the message itself in CBOR format, it contains an object with a `bytes` field which is a byte array containing the raw serialized ROS 2 message.
The `msg` object also includes `secs` and `nsecs` for the ROS time at which the message was received, which is especially useful when `use_sim_time` is set.

When using this encoding, a client application will need to know exactly how to parse the raw message.
For this it is useful to use the `/rosapi/get_topics_and_raw_types` service, which provides topic names together with their raw message definitions.

## 4. Operation specifications

### 4.1 Interface type notation

Several operations accept a `type` field that identifies a ROS interface type.
The full form is `package_name/category/TypeName`, where `category` is `msg`, `srv`, or `action` depending on the interface kind.
For example: `std_msgs/msg/String`, `std_srvs/srv/SetBool`, `nav2_msgs/action/NavigateToPose`.

The `category` component may be omitted, in which case rosbridge will infer it from context (e.g. `std_msgs/String`, `std_srvs/SetBool`).

### 4.2 Default QoS settings

Publishers created by rosbridge use reliable reliability and transient local durability, with a queue depth equal to the `queue_size` parameter (default `100`).

Subscribers created by rosbridge attempt to match the QoS of existing publishers on the topic.
When no publishers are present, the subscriber defaults to best-effort reliability and volatile durability, with a queue depth of `10`.
If all existing publishers use transient local durability, the subscriber switches to transient local durability and reliable reliability.
If any existing publisher uses best-effort reliability, the subscriber uses best-effort reliability.

### 4.3 Topic operations

#### 4.3.1 advertise (C → S)

Register the client as a publisher on a topic. This allows the server to track which clients are publishing on which topics, and to establish the topic with the correct type if it does not already exist.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"advertise"` |
| `id` | optional | string | An ID to associate with this advertisement. Useful when multiple components advertise the same topic so that each can be unadvertised independently. |
| `topic` | required | string | The name of the topic to advertise. |
| `type` | required | string | The type of the topic to advertise. |
| `latch` | optional | boolean | Whether to latch the last message published on this topic. Defaults to `false`. |
| `queue_size` | optional | integer | Size of the internal publisher queue (QoS depth policy). Defaults to `100`. |

The operation fails if either of the following is true:

- The topic already exists with a different type.
- The type specified cannot be resolved.

Current limitations:

- The protocol spawns only one publisher per topic, so if multiple clients advertise the same topic, they will share the same publisher and its associated QoS settings.
  Only the first advertisement will determine the QoS settings for that topic.

#### 4.3.2 unadvertise (C → S)

Unregister advertisement of a topic for the client.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unadvertise"` |
| `id` | optional | string | An ID of the advertisement to unregister. If provided, only the matching advertisement is removed. If omitted, all advertisements for the topic by this client are removed. |
| `topic` | required | string | The name of the topic to unadvertise. |

This operation fails if either of the following is true:

- The client has not previously advertised the topic.
- The client has already unregistered all advertisements for the topic.
- The `id` provided does not match any existing advertisement by this client for the topic.

#### 4.3.3 publish (C ↔ S)

Publish a message on a topic.

The message format is the same in both directions:

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"publish"` |
| `topic` | required | string | The name of the topic to publish on. |
| `msg` | required | object | The message being published on the topic. |

**Client → Server**

The client sends a `publish` message to push a message onto a ROS topic.

If the topic has not been advertised, the server will automatically advertise it with the provided type.
In this case, the following additional fields are also supported in the message to specify the topic type and QoS settings:

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `id` | optional | string | An ID to associate with this advertisement. |
| `type` | optional | string | The type of the topic to advertise. If omitted, the type will be inferred from the current ROS graph. |
| `latch` | optional | boolean | Whether to latch the last message published on this topic. Defaults to `false`. |
| `queue_size` | optional | integer | Size of the internal publisher queue (QoS depth policy). Defaults to `100`. |

The operation fails if the `msg` does not conform to the type of the topic.

Special cases for how the server handles the `msg` field:

- If the `msg` does not contain all fields for the topic type, then the unspecified fields are filled in with defaults.
- If the topic type has a `header` field of type `std_msgs/Header` and the client omits the `header.stamp` field, then the server will automatically populate it with the current ROS time.

**Server → Client**

The server sends a `publish` message to forward an incoming ROS topic message to a subscribed client.
This happens when a message is received on a topic that the client has previously subscribed to via the `subscribe` operation.

#### 4.3.4 subscribe (C → S)

Register a subscription to a topic to receive messages published on that topic.

It is recommended that if the client has multiple components subscribing to the same topic, that each component makes its own subscription request providing an ID.
That way, each can individually unsubscribe and rosbridge can select the correct rate at which to send messages.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"subscribe"` |
| `id` | optional | string | An ID to associate with this subscription. Useful when multiple components subscribe to the same topic so that each can be unsubscribed independently. |
| `topic` | required | string | The name of the topic to subscribe to. |
| `type` | optional | string | The expected type of the topic. If omitted, type will be inferred; if the topic does not exist the subscription will fail. |
| `throttle_rate` | optional | integer | Minimum time (in ms) that must elapse between messages being sent. Defaults to `0`. |
| `queue_length` | optional | integer | Size of the queue to buffer messages when throttled. Defaults to `0` (no queueing). When full, the oldest message is dropped in favour of the newest. |
| `fragment_size` | optional | integer | Maximum size (in bytes) a message can reach before it is fragmented. |
| `compression` | optional | string | Compression scheme for outgoing messages. Valid values: `none`, `png`, `cbor`, `cbor-raw`. |

The operation fails if either of the following is true:

- The subscription for the topic already exists with a different type.
- The type specified cannot be resolved.

If `queue_length` is specified, then messages are placed into the queue before being sent.
Messages are sent from the head of the queue.
If the queue gets full, the oldest message is removed and replaced by the newest message.

If a client has multiple subscriptions to the same topic, then messages are sent at the lowest `throttle_rate`, with the lowest `fragment_size`, and lowest `queue_length`.
It is recommended that the client provides IDs for its subscriptions to enable rosbridge to effectively choose the appropriate fragmentation size and publishing rate.

#### 4.3.5 unsubscribe (C → S)

Unsubscribe from a topic to stop receiving updates.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unsubscribe"` |
| `id` | optional | string | An ID to disassociate with this subscription. If provided, only the matching subscription is removed. If omitted, all subscriptions for the topic by this client are removed. |
| `topic` | required | string | The name of the topic to unsubscribe from. |

The operation fails if either of the following is true:

- The client has not previously subscribed to the topic.
- The client has already unregistered all subscriptions for the topic.
- The `id` provided does not match any existing subscription by this client for the topic.

### 4.4 Service operations

#### 4.4.1 advertise_service (C → S)

Advertise an external service server. Requests come to the client via `call_service`.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"advertise_service"` |
| `service` | required | string | The name of the service to advertise. |
| `type` | required | string | The advertised service type. |

The operation fails if the type specified cannot be resolved.

When a client advertises the same service a second time, the previous advertisement is replaced with the new one.

#### 4.4.2 unadvertise_service (C → S)

Stop advertising an external ROS service server.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unadvertise_service"` |
| `service` | required | string | The name of the service to unadvertise. |

The operation fails if the client has not previously advertised the service, or has already unadvertised the service.

#### 4.4.3 call_service (C ↔ S)

Invoke a service.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"call_service"` |
| `id` | required if S → C | string | An ID to associate with this service call. Will be included in the response. |
| `service` | required | string | The name of the service to call. |
| `args` | optional | object or list | The arguments to pass to the service. Can be an object with message fields or a list of field values in the order they appear in the service request definition. |
| `fragment_size` | optional | integer | (only C → S) The maximum size (in bytes) a message can reach before it is fragmented. |
| `timeout` | optional | float | (only C → S) The time, in seconds, to wait for a response from the server. |

When a client sends a `call_service` message, the operation fails if either of the following is true:

- No service servers have been advertised for the specified service.
- The `args` do not conform to the service request type.

When a server sends a `call_service` message to the client, it always contains a unique `id` field, which the client must include in the corresponding `service_response` message.

#### 4.4.4 service_response (C ↔ S)

Return a service response.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"service_response"` |
| `id` | required if C → S | string | An ID to associate with this service response. Will match the ID of the corresponding service call if it was provided. |
| `service` | required | string | The name of the service that was called. |
| `values` | conditional | object or string | When `result` is `true`, this field is **required** and must be an object containing the service's response values (conforming to the service's response message definition). When `result` is `false`, this field is **optional** and, if present, is typically a string error message. |
| `result` | required | boolean | The result of the service call. `true` indicates success (a structured `values` object is required), `false` indicates failure (an error may be conveyed via `values` or other means) |

When a client sends a `service_response` message, the operation fails if any of the following is true:

- The service has not been advertised by the client.
- The `id` field is missing or does not match any existing service call for this client.
- The `values` do not conform to the service response message definition when `result` is `true`.

### 4.5 Action operations

#### 4.5.1 advertise_action (C → S)

Advertise an external ROS action server.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"advertise_action"` |
| `action` | required | string | The name of the action to advertise. |
| `type` | required | string | The advertised action type. |

The operation fails if the type specified cannot be resolved.

When a client advertises the same action a second time, the previous advertisement is replaced with the new one.

#### 4.5.2 unadvertise_action (C → S)

Stop advertising an external ROS action server.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unadvertise_action"` |
| `action` | required | string | The name of the action to unadvertise. |

The operation fails if the client has not previously advertised the action, or has already unadvertised the action.

#### 4.5.3 send_action_goal (C ↔ S)

Send an action goal.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"send_action_goal"` |
| `id` | required if S → C | string | An ID to associate with this goal. Will be included in feedback and result messages related to this goal. |
| `action` | required | string | The name of the action to send a goal to. |
| `action_type` | required | string | The action type. |
| `args` | optional | object or list | The arguments to pass to the action goal. Can be an object with message fields or a list of field values in the order they appear in the action goal definition. |
| `feedback` | optional | boolean | Whether to send feedback messages for this goal. Defaults to `false`. |
| `fragment_size` | optional | integer | (only C → S) The maximum size (in bytes) a message can reach before it is fragmented. |

When a client sends a `send_action_goal` message, the operation fails if either of the following is true:

- No action servers have been advertised for the specified action.
- The `args` do not conform to the action goal type.

#### 4.5.4 cancel_action_goal (C ↔ S)

Cancel an action goal.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"cancel_action_goal"` |
| `id` | required | string | An ID to identify which goal to cancel. Must match the ID of an already in-progress goal. |
| `action` | required | string | The name of the action to cancel a goal for. |

When a client sends the `cancel_action_goal` message, the operation fails if either of the following is true:

- The client has not previously sent a goal for the action.
- The client has already cancelled the goal.
- The `id` provided does not match any existing goal by this client for the action.
- The goal has already completed.

#### 4.5.5 action_feedback (C ↔ S)

Report action feedback.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"action_feedback"` |
| `id` | required if C → S | string | An ID to identify which goal this feedback is for. Must match the ID of an already in-progress goal. |
| `action` | required | string | The name of the action this feedback is for. |
| `values` | required | object | The feedback values. Must conform to the feedback message definition of the action. |

When a client sends an `action_feedback` message, the operation fails if any of the following is true:

- The `id` field is missing or does not match any existing goal by this client.
- The `values` do not conform to the feedback message definition of the action.

#### 4.5.6 action_result (C ↔ S)

Report an action result.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"action_result"` |
| `id` | required if C → S | string | An ID to identify which goal this result is for. Must match the ID of an already in-progress goal. |
| `action` | required | string | The name of the action this result is for. |
| `values` | conditional | object or string | When `result` is `true`, this field is **required** and must be an object containing the action's result values (conforming to the action's result message definition). When `result` is `false`, this field is **optional** and, if present, is typically a string error message. |
| `status` | required | integer | The status of the action. This matches the enumeration in the [`action_msgs/msg/GoalStatus`](https://docs.ros2.org/latest/api/action_msgs/msg/GoalStatus.html) ROS message. |
| `result` | required | boolean | Indicates whether the action completed successfully. `true` indicates success (a structured `values` object is required), `false` indicates failure (an error may be conveyed via `values` or other means). |

When a client sends an `action_result` message, the operation fails if any of the following is true:

- The client has not previously sent a goal for the action.
- The client has already sent a result for the goal.
- The `id` provided does not match any existing goal by this client for the action.
- The `values` field is missing or does not conform to the action result message definition when `result` is `true`.

[cbor]: https://tools.ietf.org/html/rfc7049
[draft typed array tags]: https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00
[rosapi]: https://docs.ros.org/en/rolling/p/rosapi/
