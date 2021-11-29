import functools
import json
from typing import Any, Awaitable, Callable

import launch
import launch_ros
import rclpy
import rclpy.task
from autobahn.twisted.websocket import WebSocketClientFactory, WebSocketClientProtocol
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from twisted.internet import reactor
from twisted.internet.endpoints import TCP4ClientEndpoint


class TestClientProtocol(WebSocketClientProtocol):
    """
    Set message_handler to handle messages received from the server.
    """

    message_handler: Callable[[Any], None]

    def __init__(self, *args, **kwargs):
        self.received = []
        self.connected_future = rclpy.task.Future()
        self.message_handler = lambda _: None
        super().__init__(*args, **kwargs)

    def onOpen(self):
        self.connected_future.set_result(None)

    def sendJson(self, msg_dict, *, times=1):
        msg = json.dumps(msg_dict).encode("utf-8")
        for _ in range(times):
            print(f"WebSocket client sent message: {msg}")
            self.sendMessage(msg)

    def onMessage(self, payload, binary):
        print(f"WebSocket client received message: {payload}")
        self.message_handler(json.loads(payload))


def generate_test_description(ready_fn) -> launch.LaunchDescription:
    """
    Generate a launch description that runs the websocket server. Re-export this from a test file and use add_launch_test() to run the test.
    """
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


async def get_server_port(node: Node) -> int:
    """
    Returns the port which the WebSocket server is running on
    """
    client = node.create_client(GetParameters, "/rosbridge_websocket/get_parameters")
    try:
        if not client.wait_for_service(5):
            raise RuntimeError("GetParameters service not available")
        port_param = await client.call_async(GetParameters.Request(names=["actual_port"]))
        return port_param.values[0].integer_value
    finally:
        node.destroy_client(client)


async def connect_to_server(node: Node) -> TestClientProtocol:
    port = await get_server_port(node)
    factory = WebSocketClientFactory("ws://127.0.0.1:" + str(port))
    factory.protocol = TestClientProtocol

    future = rclpy.task.Future()
    future.add_done_callback(lambda _: node.executor.wake())

    def connect():
        TCP4ClientEndpoint(reactor, "127.0.0.1", port).connect(factory).addCallback(
            future.set_result
        )

    reactor.callFromThread(connect)

    protocol = await future
    protocol.connected_future.add_done_callback(lambda _: node.executor.wake())
    await protocol.connected_future  # wait for onOpen before proceeding
    return protocol


def run_websocket_test(
    node_name: str,
    test_fn: Callable[[Node, Callable[[], Awaitable[TestClientProtocol]]], Awaitable[None]],
):
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node(node_name, context=context)
    executor.add_node(node)

    async def task():
        await test_fn(node, lambda: connect_to_server(node))
        reactor.callFromThread(reactor.stop)

    future = executor.create_task(task)

    reactor.callInThread(rclpy.spin_until_future_complete, node, future, executor)
    reactor.run(installSignalHandlers=False)

    rclpy.shutdown(context=context)
    node.destroy_node()


def sleep(node: Node, duration: float) -> Awaitable[None]:
    """
    Async-compatible delay function based on a ROS timer.
    """
    future = rclpy.task.Future()

    def callback():
        future.set_result(None)
        timer.cancel()
        node.destroy_timer(timer)

    timer = node.create_timer(duration, callback)
    return future


def websocket_test(test_fn):
    """
    Decorator for tests which use a ROS node and WebSocket server and client.
    Multiple tests per file are not supported because the Twisted reactor cannot be run multiple times.
    """

    @functools.wraps(test_fn)
    def run_test(self):
        run_websocket_test(test_fn.__name__, lambda *args: test_fn(self, *args))

    return run_test


def expect_messages(count: int, description: str, logger):
    """
    Convenience function to create a Future and a message handler function which gathers results
    into a list and waits for the list to have the expected number of items.
    """
    future = rclpy.Future()
    results = []

    def handler(msg):
        logger.info(f"Received message on {description}: {msg}")
        results.append(msg)
        if len(results) == count:
            logger.info(f"Received all messages on {description}")
            future.set_result(results)
        elif len(results) > count:
            raise AssertionError(
                f"Received {len(results)} messages on {description} but expected {count}"
            )

    return future, handler
