from __future__ import annotations

import functools
import json
from typing import TYPE_CHECKING, Any, TypeVar

import launch_ros
import rclpy
from autobahn.twisted.websocket import WebSocketClientFactory, WebSocketClientProtocol
from launch.launch_description import LaunchDescription
from launch_testing.actions import ReadyToTest
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from twisted.internet import reactor
from twisted.internet.endpoints import TCP4ClientEndpoint

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from rclpy.client import Client
    from rclpy.logging import RcutilsLogger


class TestClientProtocol(WebSocketClientProtocol):
    """Set message_handler to handle messages received from the server."""

    message_handler: Callable[[Any], None]

    def __init__(self, *args: Any, **kwargs: Any) -> None:  # noqa: ANN401
        self.connected_future: Future[None] = Future()
        self.message_handler = lambda _: None
        super().__init__(*args, **kwargs)

    def onOpen(self) -> None:
        self.connected_future.set_result(None)

    def sendJson(self, msg_dict: dict[str, Any], *, times: int = 1) -> None:
        msg = json.dumps(msg_dict).encode("utf-8")
        for _ in range(times):
            print(f"WebSocket client sent message: {msg!r}")
            self.sendMessage(msg)

    def onMessage(self, payload: str, binary: bool) -> None:
        print(f"WebSocket client received message: {payload}")
        self.message_handler(payload if binary else json.loads(payload))


def _generate_node() -> launch_ros.actions.Node:
    return launch_ros.actions.Node(
        executable="rosbridge_websocket",
        package="rosbridge_server",
        parameters=[{"port": 0}],
    )


def generate_test_description() -> LaunchDescription:
    """
    Generate a launch description that runs the websocket server.

    Re-export this from a test file and use add_launch_test() to run the test.
    """
    return LaunchDescription([_generate_node(), ReadyToTest()])


async def get_server_port(node: Node) -> int:
    """Return the port which the WebSocket server is running on."""
    client: Client = node.create_client(GetParameters, "/rosbridge_websocket/get_parameters")  # type: ignore[arg-type]
    try:
        if not client.wait_for_service(5):
            msg = "GetParameters service not available"
            raise RuntimeError(msg)
        port_param = await client.call_async(GetParameters.Request(names=["actual_port"]))
        assert port_param is not None
        return port_param.values[0].integer_value
    finally:
        node.destroy_client(client)


async def connect_to_server(node: Node) -> TestClientProtocol:
    port = await get_server_port(node)
    factory = WebSocketClientFactory("ws://127.0.0.1:" + str(port))
    factory.protocol = TestClientProtocol

    future: Future = Future()
    executor = node.executor
    assert executor is not None
    future.add_done_callback(lambda _: executor.wake())

    def connect() -> None:
        TCP4ClientEndpoint(reactor, "127.0.0.1", port).connect(factory).addCallback(
            future.set_result
        )

    reactor.callFromThread(connect)  # type: ignore[attr-defined]

    protocol: TestClientProtocol | None = await future
    assert protocol is not None
    protocol.connected_future.add_done_callback(lambda _: executor.wake())
    await protocol.connected_future  # wait for onOpen before proceeding
    return protocol


def run_websocket_test(
    node_name: str,
    test_fn: Callable[[Node, Callable[[], Awaitable[TestClientProtocol]]], Awaitable[None]],
) -> None:
    context = rclpy.Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    node = Node(node_name, context=context)
    executor.add_node(node)

    async def task() -> None:
        await test_fn(node, lambda: connect_to_server(node))
        reactor.callFromThread(reactor.stop)  # type: ignore[attr-defined]

    future = executor.create_task(task)

    reactor.callInThread(executor.spin_until_future_complete, future)  # type: ignore[attr-defined]
    reactor.run(installSignalHandlers=False)  # type: ignore[attr-defined]

    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown(context=context)


def sleep(node: Node, duration: float) -> Awaitable[None]:
    """
    Sleep for a given duration in seconds.

    Async-compatible delay function based on a ROS timer.
    """
    future: Future = Future()

    def callback() -> None:
        future.set_result(None)
        timer.cancel()
        node.destroy_timer(timer)

    timer = node.create_timer(duration, callback)
    return future


SelfT = TypeVar("SelfT")


def websocket_test(
    test_fn: Callable[[SelfT, Node, Callable[[], Awaitable[TestClientProtocol]]], Awaitable[None]],
) -> Callable[[SelfT], None]:
    """
    Decorate tests which use a ROS node and WebSocket server and client.

    Multiple tests per file are not supported because the Twisted reactor cannot be run multiple times.
    """

    @functools.wraps(test_fn)
    def run_test(self: SelfT) -> None:
        run_websocket_test(test_fn.__name__, lambda *args: test_fn(self, *args))

    return run_test


MsgT = TypeVar("MsgT")


def expect_messages(
    count: int, description: str, logger: RcutilsLogger
) -> tuple[Future[list[MsgT]], Callable[[MsgT], None]]:
    """
    Expect a specific number of messages.

    Convenience function to create a Future and a message handler function which gathers results
    into a list and waits for the list to have the expected number of items.
    """
    future: Future[list[MsgT]] = Future()
    results: list[MsgT] = []

    def handler(msg: MsgT) -> None:
        logger.info(f"Received message on {description}: {msg}")  # noqa: G004
        results.append(msg)
        if len(results) == count:
            logger.info(f"Received all messages on {description}")  # noqa: G004
            future.set_result(results)
        elif len(results) > count:
            err_msg = f"Received {len(results)} messages on {description} but expected {count}"
            raise AssertionError(err_msg)

    return future, handler
