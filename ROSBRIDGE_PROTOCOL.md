# rosbridge v2 Protocol Specification <!-- omit in toc -->

This document defines the rosbridge protocol and its supported operations.
The protocol is built around structured message objects (e.g., JSON or CBOR) with an `op` field that identifies the
operation being performed. The protocol is transport-agnostic and can be carried over WebSockets, TCP, or other suitable transports.

This document also describes the intended direction of the rosbridge server implementation. The default server implementation uses WebSockets and separates message parsing from the underlying transport so protocol operations remain easy to extend.

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
  - [4.1 Topic operations](#41-topic-operations)
    - [4.1.1 advertise (C → S)](#411-advertise-c--s)
    - [4.1.2 unadvertise (C → S)](#412-unadvertise-c--s)
    - [4.1.3 publish (C ↔ S)](#413-publish-c--s)
    - [4.1.4 subscribe (C → S)](#414-subscribe-c--s)
    - [4.1.5 unsubscribe (C → S)](#415-unsubscribe-c--s)
  - [4.2 Service operations](#42-service-operations)
    - [4.2.1 advertise\_service (C → S)](#421-advertise_service-c--s)
    - [4.2.2 unadvertise\_service (C → S)](#422-unadvertise_service-c--s)
    - [4.2.3 call\_service (C ↔ S)](#423-call_service-c--s)
    - [4.2.4 service\_response (C ↔ S)](#424-service_response-c--s)
  - [4.3 Action operations](#43-action-operations)
    - [4.3.1 advertise\_action (C → S)](#431-advertise_action-c--s)
    - [4.3.2 unadvertise\_action (C → S)](#432-unadvertise_action-c--s)
    - [4.3.3 send\_action\_goal (C ↔ S)](#433-send_action_goal-c--s)
    - [4.3.4 cancel\_action\_goal (C ↔ S)](#434-cancel_action_goal-c--s)
    - [4.3.5 action\_feedback (C ↔ S)](#435-action_feedback-c--s)
    - [4.3.6 action\_result (C ↔ S)](#436-action_result-c--s)

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

Some `C ↔ S` operations are valid in either direction depending on which side has advertised the corresponding topic, service, or action interface.

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

While CBOR encodes the entire message as CBOR, sometimes it is desirable to get the raw binary message in the
ROS 2 serialized message format.

This can be useful in several cases:

- Your application already knows how to parse raw ROS 2 message data or data stored in ROS 2 bag files,
  which means that you can use consistent code paths for both recorded and live messages.
- You want to parse messages as late as possible, or in parallel, e.g. only in the thread or WebWorker that cares about the message.
  Delaying the parsing of the message means that moving or copying the message to the thread is cheaper when its in binary form, since no serialization between threads is necessary.
- You only care about part of the message, and don't need to parse the rest of it.
- You really care about performance; no conversion between the ROS 2 binary message format and CBOR is done in
  the rosbridge server.

The format is similar to CBOR above, but instead of the `msg` field containing the message itself in CBOR format,
it contains an object with a `bytes` field which is a byte array containing the raw serialized ROS 2 message.
The `msg` object also includes `secs` and `nsecs` for the ROS time at which the message was received, which is especially useful when `use_sim_time` is set.

When using this encoding, a client application will need to know exactly how to parse the raw message.
For this it is useful to use the `/rosapi/get_topics_and_raw_types` service, which provides topic names together with their raw message definitions.

## 4. Operation specifications

These rosbridge messages interact with ROS, and correspond roughly to the messages that already exist in the current version of rosbridge.

### 4.1 Topic operations

#### 4.1.1 advertise (C → S)

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

#### 4.1.2 unadvertise (C → S)

Unregister advertisement of a topic for the client.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unadvertise"` |
| `id` | optional | string | An ID to disassociate with this advertisement. If provided, only the matching advertisement is removed. If omitted, all advertisements for the topic by this client are removed. |
| `topic` | required | string | The name of the topic to unadvertise. |

This operation fails if either of the following is true:

- The client has not previously advertised the topic.
- The client has already unregistered all advertisements for the topic.
- The `id` provided does not match any existing advertisement by this client for the topic.

#### 4.1.3 publish (C ↔ S)

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

#### 4.1.4 subscribe (C → S)

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

If a client has multiple subscriptions to the same topic, then messages are sent at the lowest `throttle_rate`, with the lowest `fragment_size`, and highest `queue_length`.
It is recommended that the client provides IDs for its subscriptions to enable rosbridge to effectively choose the appropriate fragmentation size and publishing rate.

#### 4.1.5 unsubscribe (C → S)

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

### 4.2 Service operations

#### 4.2.1 advertise_service (C → S)

Advertise an external service server. Requests come to the client via `call_service`.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"advertise_service"` |
| `service` | required | string | The name of the service to advertise. |
| `type` | required | string | The advertised service type. |

#### 4.2.2 unadvertise_service (C → S)

Stop advertising an external ROS service server.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unadvertise_service"` |
| `service` | required | string | The name of the service to unadvertise. |

#### 4.2.3 call_service (C ↔ S)

Invoke a service.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"call_service"` |
| `id` | optional | string | An ID to associate with this service call. Will be included in the response. |
| `service` | required | string | The name of the service to call. |
| `args` | optional | object or list | The arguments to pass to the service. Can be an object with message fields or a list of field values in the order they appear in the service request definition. |
| `fragment_size` | optional | integer | (only C → S) The maximum size (in bytes) a message can reach before it is fragmented. |
| `timeout` | optional | float | (only C → S) The time, in seconds, to wait for a response from the server. |

#### 4.2.4 service_response (C ↔ S)

Return a service response.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"service_response"` |
| `id` | optional | string | An ID to associate with this service response. Will match the ID of the corresponding service call if it was provided. |
| `service` | required | string | The name of the service that was called. |
| `values` | required | object or string | The return values from the service or an error message if the service call failed. |
| `result` | required | boolean | The result of the service call. `true` indicates success, `false` indicates failure. |

### 4.3 Action operations

#### 4.3.1 advertise_action (C → S)

Advertise an external ROS action server.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"advertise_action"` |
| `action` | required | string | The name of the action to advertise. |
| `type` | required | string | The advertised action type. |

#### 4.3.2 unadvertise_action (C → S)

Stop advertising an external ROS action server.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"unadvertise_action"` |
| `action` | required | string | The name of the action to unadvertise. |

#### 4.3.3 send_action_goal (C ↔ S)

Send an action goal.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"send_action_goal"` |
| `id` | optional | string | An ID to associate with this goal. Will be included in feedback and result messages related to this goal. |
| `action` | required | string | The name of the action to send a goal to. |
| `action_type` | required | string | The action type. |
| `args` | optional | object or list | The arguments to pass to the action goal. Can be an object with message fields or a list of field values in the order they appear in the action goal definition. |
| `feedback` | optional | boolean | Whether to send feedback messages for this goal. Defaults to `false`. |
| `fragment_size` | optional | integer | (only C → S) The maximum size (in bytes) a message can reach before it is fragmented. |

#### 4.3.4 cancel_action_goal (C ↔ S)

Cancel an action goal.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"cancel_action_goal"` |
| `id` | required | string | An ID to identify which goal to cancel. Must match the ID of an already in-progress goal. |
| `action` | required | string | The name of the action to cancel a goal for. |

#### 4.3.5 action_feedback (C ↔ S)

Report action feedback.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"action_feedback"` |
| `id` | required | string | An ID to identify which goal this feedback is for. Must match the ID of an already in-progress goal. |
| `action` | required | string | The name of the action this feedback is for. |
| `values` | required | object | The feedback values. Must conform to the feedback message definition of the action. |

#### 4.3.6 action_result (C ↔ S)

Report an action result.

| Field | Required | Type | Description |
|-------|----------|------|-------------|
| `op` | required | string | Must be `"action_result"` |
| `id` | required | string | An ID to identify which goal this result is for. Must match the ID of an already in-progress goal. |
| `action` | required | string | The name of the action this result is for. |
| `values` | required | object or string | The result values from the action or an error message if the action failed. |
| `status` | required | integer | The status of the action. This matches the enumeration in the [`action_msgs/msg/GoalStatus`](https://docs.ros2.org/latest/api/action_msgs/msg/GoalStatus.html) ROS message. |
| `result` | required | boolean | The result of the action. `true` indicates success, `false` indicates failure. |

[cbor]: https://tools.ietf.org/html/rfc7049
[draft typed array tags]: https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00
