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
    - [4.1.1 advertise](#411-advertise)
    - [4.1.2 unadvertise](#412-unadvertise)
    - [4.1.3 publish](#413-publish)
    - [4.1.4 subscribe](#414-subscribe)
    - [4.1.5 unsubscribe](#415-unsubscribe)
  - [4.2 Service operations](#42-service-operations)
    - [4.2.1 advertise\_service](#421-advertise_service)
    - [4.2.2 unadvertise\_service](#422-unadvertise_service)
    - [4.2.3 call\_service](#423-call_service)
    - [4.2.4 unadvertise\_service](#424-unadvertise_service)
    - [4.2.5 service\_response](#425-service_response)
  - [4.3 Action operations](#43-action-operations)
    - [4.3.1 advertise\_action](#431-advertise_action)
    - [4.3.2 unadvertise\_action](#432-unadvertise_action)
    - [4.3.3 send\_action\_goal](#433-send_action_goal)
    - [4.3.4 cancel\_action\_goal](#434-cancel_action_goal)
    - [4.3.5 action\_feedback](#435-action_feedback)
    - [4.3.6 action\_result](#436-action_result)
- [5 Further considerations](#5-further-considerations)
  - [5.1 Rosbridge pseudo-services](#51-rosbridge-pseudo-services)
  - [5.2 Sampling](#52-sampling)
  - [5.3 Latching](#53-latching)

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

If an `id` is provided with a message to the server, then related response messages will typically contain that ID as well.
Log messages caused by this operation will also include the ID, so that clients can easily associate log messages with the operation that caused them.

Semantically, the `id` does not identify a single message.
Instead, it identifies an interaction, which may consist of multiple back-and-forth operations.

## 2. Operation summary

The rosbridge protocol defines a number of different operations.

Direction legend:

- **C -> S**: client to server
- **S -> C**: server to client
- **C \<-> S**: either direction

Some `C <-> S` operations are valid in either direction depending on which side has advertised the corresponding topic, service, or action interface.

Message compression / transformation:

- **fragment** - C \<-> S - part of a fragmented message
- **png** - S -> C - a message compressed as a PNG image

Topic operations:

- **advertise** - C -> S - advertise that the client will publish on a topic
- **unadvertise** - C -> S - stop advertising that the client will publish on a topic
- **publish** - C \<-> S - publish a message on a topic
- **subscribe** - C -> S - request topic updates
- **unsubscribe** - C -> S - stop topic updates

Service operations:

- **advertise_service** - C -> S - advertise an external service server
- **unadvertise_service** - C -> S - stop advertising an external service server
- **call_service** - C \<-> S - invoke a service
- **service_response** - C \<-> S - return a service response

Action operations:

- **advertise_action** - C -> S - advertise an external action server
- **unadvertise_action** - C -> S - stop advertising an external action server
- **send_action_goal** - C \<-> S - send an action goal
- **cancel_action_goal** - C \<-> S - cancel an action goal
- **action_feedback** - C \<-> S - report action feedback
- **action_result** - C \<-> S - report an action result

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

```json
{
  "op": "fragment",
  "id": <string>,
  "data": <string>,
  "num": <int>,
  "total": <int>
}
```

- **id** - an id is required for fragmented messages, in order to identify corresponding fragments for the fragmented message.
- **data** - a fragment of data that, when combined with other fragments of data, makes up another message.
- **num** - the index of the fragment in the message.
- **total** - the total number of fragments.

To fragment a message, its serialized payload is taken and split up into multiple substrings or byte arrays.
For each chunk, a fragment message is constructed, with the data field of the fragment populated by the chunk.

To reconstruct an original message, the data fields of the fragments are concatenated, resulting in the serialized payload of the original message.

### 3.3 PNG compression ( _png_ ) [experimental]

Some messages (such as images and maps) can be extremely large, and for efficiency reasons we may wish to transfer them as PNG-encoded bytes.

```json
{
  "op": "png",
  "data": <string>
}
```

- **data** – a PNG-encoded message.

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

#### 4.1.1 advertise

If you wish to advertise that you are or will be publishing a topic, then use the advertise command.

```json
{
  "op": "advertise",
  (optional) "id": <string>,
  "topic": <string>,
  "type": <string>
}
```

- **topic** – the string name of the topic to advertise

- **type** – the string type to advertise for the topic

  - If the topic does not already exist, and the type specified is a valid
    type, then the topic will be established with this type.
  - If the topic already exists with a different type, an error status message
    is sent and this message is dropped.
  - If the topic already exists with the same type, the sender of this message
    is registered as another publisher.
  - If the topic doesn't already exist but the type cannot be resolved, then
    an error status message is sent and this message is dropped.

#### 4.1.2 unadvertise

This stops advertising that you are publishing a topic.

```json
{ "op": "unadvertise",
  (optional) "id": <string>,
  "topic": <string>
}
```

- **topic** – the string name of the topic being unadvertised

  - If the topic does not exist, a warning status message is sent and this
    message is dropped
  - If the topic exists and there are still clients left advertising it,
    rosbridge will continue to advertise it until all of them have unadvertised
  - If the topic exists but rosbridge is not advertising it, a warning status
    message is sent and this message is dropped

#### 4.1.3 publish

The publish message is used to send data on a topic.

```json
{ "op": "publish",
  (optional) "id": <string>,
  "msg": <message_object>
}
```

The publish command publishes a message on a topic.

- **topic** - the string name of the topic to publish to

- **msg** - the message to publish on the topic

  - If the topic does not exist, then an error status message is sent and this
    message is dropped
  - If the msg does not conform to the type of the topic, then an error status
    message is sent and this message is dropped
  - If the msg is a subset of the type of the topic, then a warning status
    message is sent and the unspecified fields are filled in with defaults

Special case: if the type being published has a 'header' field, then the client
can optionally omit the header from the msg. If this happens, rosbridge will
automatically populate the header with a frame id of "" and the timestamp as
the current time. Alternatively, just the timestamp field can be omitted, and
then the current time will be automatically inserted.

#### 4.1.4 subscribe

```json
{ "op": "subscribe",
  (optional) "id": <string>,
  "topic": <string>,
  (optional) "type": <string>,
  (optional) "throttle_rate": <int>,
  (optional) "queue_length": <int>,
  (optional) "fragment_size": <int>,
  (optional) "compression": <string>
}
```

This command subscribes the client to the specified topic. It is recommended
that if the client has multiple components subscribing to the same topic, that
each component makes its own subscription request providing an ID. That way,
each can individually unsubscribe and rosbridge can select the correct rate at
which to send messages.

- **type** – the (expected) type of the topic to subscribe to. If left off,
  type will be inferred, and if the topic doesn't exist then the command to
  subscribe will fail
- **topic** – the name of the topic to subscribe to
- **throttle_rate** – the minimum amount of time (in ms) that must elapse
  between messages being sent. Defaults to 0
- **queue_length** – the size of the queue to buffer messages. Messages are
  buffered as a result of the throttle_rate. Defaults to 0 (no queueing).
- **id** – if specified, then this specific subscription can be unsubscribed
  by referencing the ID.
- **fragment_size** – the maximum size that a message can take before it is to
  be fragmented.
- **compression** – an optional string to specify the compression scheme to be
  used on messages. Valid values are "none", "png", "cbor", and "cbor-raw".

If queue_length is specified, then messages are placed into the queue before
being sent. Messages are sent from the head of the queue. If the queue gets
full, the oldest message is removed and replaced by the newest message.

If a client has multiple subscriptions to the same topic, then messages are
sent at the lowest throttle_rate, with the lowest fragmentation size, and
highest queue_length. It is recommended that the client provides IDs for its
subscriptions, to enable rosbridge to effectively choose the appropriate
fragmentation size and publishing rate.

#### 4.1.5 unsubscribe

```json
{ "op": "unsubscribe",
  (optional) "id": <string>,
  "topic": <string>
}
```

- **topic** – the name of the topic to unsubscribe from
- **id** – an id of the subscription to unsubscribe

If an id is provided, then only the corresponding subscription is unsubscribed.
If no ID is provided, then all subscriptions are unsubscribed.

### 4.2 Service operations

#### 4.2.1 advertise_service

```json
{ "op": "advertise_service",
  "type": <string>,
  "service": <string>
}
```

Advertises an external ROS service server. Requests come to the client via Call Service.

- **service** – the name of the service to advertise
- **type** – the advertised service message type

#### 4.2.2 unadvertise_service

```json
{ "op": "unadvertise_service",
  "service": <string>
}
```

#### 4.2.3 call_service

Calls a ROS service.

```json
{ "op": "call_service",
  (optional) "id": <string>,
  "service": <string>,
  (optional) "args": <list<json>>,
  (optional) "fragment_size": <int>,
  (optional) "compression": <string>,
  (optional) "timeout": <float>
}
```

- **service** – the name of the service to call
- **args** – if the service has no args, then args does not have to be
  provided, though an empty list is equally acceptable. Args should be a list
  of json objects representing the arguments to the service
- **id** – an optional id to distinguish this service call
- **fragment_size** – the maximum size that the response message can take
  before it is fragmented
- **compression** – an optional string to specify the compression scheme to be
  used on messages. Valid values are "none" and "png"
- **timeout** – the time, in seconds, to wait for a response from the server

#### 4.2.4 unadvertise_service

Stops advertising an external ROS service server

```json
{ "op": "unadvertise_service",
  "service": <string>
}
```

- **service** – the name of the service to unadvertise

#### 4.2.5 service_response

A response to a ROS service call.

```json
{ "op": "service_response",
  (optional) "id": <string>,
  "service": <string>,
  (optional) "values": <list<json>>,
  "result": <boolean>
}
```

- **service** – the name of the service that was called
- **values** – the return values. If the service had no return values, then
  this field can be omitted (and will be by the rosbridge server)
- **id** – if an ID was provided to the service request, then the service
  response will contain the ID
- **result** - return value of service callback. true means success, false failure.

### 4.3 Action operations

#### 4.3.1 advertise_action

Advertises an external ROS action server.

```json
{ "op": "advertise_action",
  "type": <string>,
  "action": <string>
}
```

Goals come to the client via the Send Action Goal capability.

- **action** – the name of the action to advertise
- **type** – the advertised action message type

#### 4.3.2 unadvertise_action

```json
{ "op": "unadvertise_action",
  "action": <string>
}
```

#### 4.3.3 send_action_goal

Sends a goal to a ROS action server.

```json
{ "op": "send_action_goal",
  (optional) "id": <string>,
  "action": <string>,
  "action_type": <string>,
  (optional) "args": <list<json>>,
  (optional) "feedback": <boolean>,
  (optional) "fragment_size": <int>,
  (optional) "compression": <string>
}
```

- **action** – the name of the action to send a goal to
- **action_type** – the action message type
- **args** – if the goal has no args, then args does not have to be
  provided, though an empty list is equally acceptable. Args should be a list of json objects representing the arguments to the service.
- **feedback** – if true, sends feedback messages over rosbridge. Defaults to false.
- **id** – an optional id to distinguish this goal handle
- **fragment_size** – the maximum size that the result and feedback messages can take before they are fragmented
- **compression** – an optional string to specify the compression scheme to be used on messages. Valid values are "none" and "png"

#### 4.3.4 cancel_action_goal

Cancels an action goal.

```json
{ "op": "cancel_action_goal",
  "id": <string>,
  "action": <string>
}
```

The `id` field must match an already in-progress goal.

#### 4.3.5 action_feedback

Used to send action feedback for a specific goal handle.

```json
{ "op": "action_feedback",
  "id": <string>,
  "action": <string>,
  "values": <json>
}
```

The `id` field must match an already in-progress goal.

#### 4.3.6 action_result

A result for a ROS action.

```json
{ "op": "action_result",
  "id": <string>,
  "action": <string>,
  "values": <json>,
  "status": <int>,
  "result": <boolean>
}
```

- **action** – the name of the action that was executed
- **id** – if an ID was provided to the action goal, then the action result will contain the ID
- **values** – the result values. If the service had no return values, then
  this field can be omitted (and will be by the rosbridge server)
- **status** - return status of the action. This matches the enumeration in the [`action_msgs/msg/GoalStatus`](https://docs.ros2.org/latest/api/action_msgs/msg/GoalStatus.html) ROS message.
- **result** - return value of action. True means success, false failure.

______________________________________________________________________

## 5 Further considerations

Further considerations for the rosbridge protocol are listed below.

### 5.1 Rosbridge pseudo-services

Rosbridge no longer provides the ROS-api introspection pseudo services that it
previously did. These are, for example rosbridge/topics and rosbridge/services.
Instead, these services are provided as proper ROS services by the new rosapi
package.

### 5.2 Sampling

It has been suggested that rosbridge may be extended to provide an operation to
sample a single message from a topic.

### 5.3 Latching

Rosbridge will support messages that were latched to topics internally in ROS.
It is possible that the publish opcode will be extended so that remote clients
can latch messages too.

[cbor]: https://tools.ietf.org/html/rfc7049
[draft typed array tags]: https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00
