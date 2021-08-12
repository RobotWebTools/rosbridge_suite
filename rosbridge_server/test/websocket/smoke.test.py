#!/usr/bin/env python
import json
import sys
import unittest

from autobahn.twisted.websocket import WebSocketClientFactory, WebSocketClientProtocol
from twisted.internet import reactor
from twisted.internet.endpoints import TCP4ClientEndpoint
from twisted.python import log

import launch
import launch.actions
import launch_ros
import launch_ros.actions
import rclpy
import rclpy.task
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

log.startLogging(sys.stderr)

# For consistency, the number of messages must not exceed the the protocol
# Subscriber queue_size.
NUM_MSGS = 10
MSG_SIZE = 10
A_TOPIC = "/a_topic"
B_TOPIC = "/b_topic"
A_STRING = "A" * MSG_SIZE
B_STRING = "B" * MSG_SIZE
WARMUP_DELAY = 1.0  # seconds
TIME_LIMIT = 5.0  # seconds


def generate_test_description(ready_fn):
    try:
        node = launch_ros.actions.Node(
            executable="rosbridge_websocket",
            package="rosbridge_server",
            parameters=[{"port": 0}],
        )
    except TypeError:
        # Deprecated keyword arg node_executable: https://github.com/ros2/launch_ros/pull/140
        node = launch_ros.actions.Node(
            node_executable="rosbridge_websocket",
            package="rosbridge_server",
            parameters=[{"port": 0}],
        )

    return launch.LaunchDescription(
        [
            node,
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]
    )


class TestClientProtocol(WebSocketClientProtocol):
    def __init__(self, *args, **kwargs):
        self.received = []
        self.connected_future = rclpy.task.Future()
        super().__init__(*args, **kwargs)

    def onOpen(self):
        self.connected_future.set_result(None)

    def sendDict(self, msg_dict, times=1):
        msg = json.dumps(msg_dict).encode("utf-8")
        for _ in range(times):
            print(f"WebSocket client sent message: {msg}")
            self.sendMessage(msg)

    def onMessage(self, payload, binary):
        print(f"WebSocket client received message: {payload}")
        self.received.append(payload)


class TestWebsocketSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.node = rclpy.create_node("websocket_smoke_test")
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    async def get_server_port(self):
        """
        Returns the port which the WebSocket server is running on
        """
        client = self.node.create_client(
            GetParameters, "/rosbridge_websocket/get_parameters"
        )
        try:
            if not client.wait_for_service(5):
                raise RuntimeError("GetParameters service not available")
            port_param = await client.call_async(
                GetParameters.Request(names=["actual_port"])
            )
            return port_param.values[0].integer_value
        finally:
            self.node.destroy_client(client)

    async def connect_to_server(self):
        port = await self.get_server_port()
        factory = WebSocketClientFactory("ws://127.0.0.1:" + str(port))
        factory.protocol = TestClientProtocol

        future = rclpy.task.Future()
        future.add_done_callback(lambda _: self.executor.wake())

        def connect():
            TCP4ClientEndpoint(reactor, "127.0.0.1", port).connect(factory).addCallback(
                future.set_result
            )

        reactor.callFromThread(connect)

        protocol = await future
        protocol.connected_future.add_done_callback(lambda _: self.executor.wake())
        await protocol.connected_future  # wait for onOpen before proceeding
        return protocol

    def sleep(self, duration):
        future = rclpy.task.Future()

        def callback():
            future.set_result(None)
            timer.cancel()
            self.node.destroy_timer(timer)

        timer = self.node.create_timer(duration, callback)
        return future

    def test_smoke(self):
        ros_received = []
        sub_a = self.node.create_subscription(
            String, A_TOPIC, ros_received.append, NUM_MSGS
        )
        pub_b = self.node.create_publisher(String, B_TOPIC, NUM_MSGS)

        async def run_test():
            print("Connecting to server...")
            ws_client = await self.connect_to_server()
            print("Connected!")

            ws_client.sendDict(
                {
                    "op": "subscribe",
                    "topic": B_TOPIC,
                    "type": "std_msgs/String",
                    "queue_length": 0,  # Test the roslib default.
                }
            )
            ws_client.sendDict(
                {
                    "op": "advertise",
                    "topic": A_TOPIC,
                    "type": "std_msgs/String",
                }
            )
            ws_client.sendDict(
                {
                    "op": "publish",
                    "topic": A_TOPIC,
                    "msg": {
                        "data": A_STRING,
                    },
                },
                NUM_MSGS,
            )

            await self.sleep(WARMUP_DELAY)

            for _ in range(NUM_MSGS):
                pub_b.publish(String(data=B_STRING))

            await self.sleep(TIME_LIMIT)

            reactor.callFromThread(reactor.stop)
            return ws_client.received

        future = self.executor.create_task(run_test)
        reactor.callInThread(
            rclpy.spin_until_future_complete, self.node, future, self.executor
        )
        reactor.run(installSignalHandlers=False)

        ws_received = future.result()
        for received in ws_received:
            msg = json.loads(received)
            self.assertEqual("publish", msg["op"])
            self.assertEqual(B_TOPIC, msg["topic"])
            self.assertEqual(B_STRING, msg["msg"]["data"])
        self.assertEqual(NUM_MSGS, len(ws_received))

        for msg in ros_received:
            self.assertEqual(A_STRING, msg.data)
        self.assertEqual(NUM_MSGS, len(ros_received))

        self.node.destroy_subscription(sub_a)
        self.node.destroy_publisher(pub_b)
