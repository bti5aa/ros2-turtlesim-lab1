
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
from lab1_interfaces.srv import ResetTurtle


class ResetServer(Node):
    def __init__(self):
        super().__init__('reset_server')

        self.group = ReentrantCallbackGroup()

        self.reset_client = self.create_client(
            Empty,
            '/reset',
            callback_group=self.group
        )

        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute',
            callback_group=self.group
        )

        self.server = self.create_service(
            ResetTurtle,
            '/reset_turtle',
            self.reset_callback,
            callback_group=self.group
        )

        self.get_logger().info('Reset service ready')

    def wait_for_future(self, future, timeout_sec=5.0):
        event = threading.Event()

        def on_done(_):
            event.set()

        future.add_done_callback(on_done)

        if not event.wait(timeout=timeout_sec):
            raise TimeoutError('Service call timed out')

        result = future.result()

        if result is None:
            raise RuntimeError('Service returned no result')

        return result

    def reset_callback(self, request, response):
        try:
            if request.clear_screen:
                if not self.reset_client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError('/reset service unavailable')

                self.wait_for_future(
                    self.reset_client.call_async(Empty.Request())
                )

            if not self.teleport_client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(
                    '/turtle1/teleport_absolute unavailable'
                )

            teleport_request = TeleportAbsolute.Request()
            teleport_request.x = 5.544445
            teleport_request.y = 5.544445
            teleport_request.theta = 0.0

            self.wait_for_future(
                self.teleport_client.call_async(teleport_request)
            )

            response.success = True
            response.message = 'Screen cleared and turtle reset'
            self.get_logger().info(response.message)

        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(
                f'Reset failed: {response.message}'
            )

        return response


def main(args=None):
    rclpy.init(args=args)

    node = ResetServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

