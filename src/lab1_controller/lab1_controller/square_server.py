
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from turtlesim.msg import Pose

from lab1_interfaces.action import DrawSquare, GoToPose
from lab1_interfaces.srv import ResetTurtle


class SquareServer(Node):
    def __init__(self):
        super().__init__('square_server')

        self.group = ReentrantCallbackGroup()
        self.current_pose = None

        self.declare_parameter('default_side_length', 2.0)
        self.declare_parameter('default_speed', 1.0)

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.group
        )

        self.go_client = ActionClient(
            self, GoToPose, '/goToPose',
            callback_group=self.group
        )

        self.reset_client = self.create_client(
            ResetTurtle, '/reset_turtle',
            callback_group=self.group
        )

        self.params_client = self.create_client(
            SetParameters,
            '/go_to_pose_server/set_parameters',
            callback_group=self.group
        )

        self.server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=self.group
        )

        self.get_logger().info('Square action server ready')

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, goal):
        if not math.isfinite(goal.side_length) or goal.side_length <= 0.0:
            return GoalResponse.REJECT

        if not math.isfinite(goal.speed) or goal.speed <= 0.0:
            return GoalResponse.REJECT

        if goal.side_length > 4.0:
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def wait_for_future(self, future, timeout_sec=30.0):
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)

        if not rclpy.ok():
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'Future failed: {exc}')
            return None

    def set_linear_speed(self, value):
        if not self.params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Parameter service unavailable')
            return False

        parameter = Parameter(
            name='max_linear_speed',
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value)
            )
        )

        request = SetParameters.Request()
        request.parameters = [parameter]

        response = self.wait_for_future(
            self.params_client.call_async(request),
            timeout_sec=5.0
        )

        if response is None or not response.results:
            return False

        if not response.results[0].successful:
            self.get_logger().error(
                response.results[0].reason
            )
            return False

        return True

    def execute_callback(self, goal_handle):
        result = DrawSquare.Result()
        feedback = DrawSquare.Feedback()

        side = goal_handle.request.side_length
        speed_factor = goal_handle.request.speed

        original_speed = None
        speed_changed = False

        try:
            if not self.reset_client.wait_for_service(timeout_sec=5.0):
                result.success = False
                result.message = 'Reset service unavailable'
                goal_handle.abort()
                return result

            if not self.go_client.wait_for_server(timeout_sec=5.0):
                result.success = False
                result.message = 'goToPose action unavailable'
                goal_handle.abort()
                return result

            if not self.params_client.wait_for_service(timeout_sec=5.0):
                result.success = False
                result.message = 'Parameter service unavailable'
                goal_handle.abort()
                return result

            # Read the controller's current speed before changing it.
            get_parameters_client = self.create_client(
                __import__('rcl_interfaces.srv', fromlist=['GetParameters']).GetParameters,
                '/go_to_pose_server/get_parameters',
                callback_group=self.group
            )

            if not get_parameters_client.wait_for_service(timeout_sec=5.0):
                result.success = False
                result.message = 'Could not read original speed'
                goal_handle.abort()
                return result

            GetParameters = __import__(
                'rcl_interfaces.srv',
                fromlist=['GetParameters']
            ).GetParameters

            get_request = GetParameters.Request()
            get_request.names = ['max_linear_speed']

            get_response = self.wait_for_future(
                get_parameters_client.call_async(get_request),
                timeout_sec=5.0
            )

            if (
                get_response is None
                or not get_response.values
                or get_response.values[0].type
                != ParameterType.PARAMETER_DOUBLE
            ):
                result.success = False
                result.message = 'Could not read original speed'
                goal_handle.abort()
                return result

            original_speed = get_response.values[0].double_value

            # speed is a factor: 0.5 = half max speed, 1.0 = max speed.
            effective_speed = min(
                original_speed,
                max(0.05, original_speed * speed_factor)
            )

            if not self.set_linear_speed(effective_speed):
                result.success = False
                result.message = 'Could not set drawing speed'
                goal_handle.abort()
                return result

            speed_changed = True

            self.current_pose = None

            reset_response = self.wait_for_future(
                self.reset_client.call_async(
                    ResetTurtle.Request(clear_screen=True)
                )
            )

            if reset_response is None or not reset_response.success:
                result.success = False
                result.message = 'Reset failed or timed out'
                goal_handle.abort()
                return result

            deadline = time.monotonic() + 5.0
            while (
                rclpy.ok()
                and self.current_pose is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)

            if self.current_pose is None:
                result.success = False
                result.message = 'Could not read turtle pose after reset'
                goal_handle.abort()
                return result

            start_x = self.current_pose.x
            start_y = self.current_pose.y

            margin = 0.5
            if (
                start_x + side > 11.0 - margin
                or start_y + side > 11.0 - margin
                or start_x < margin
                or start_y < margin
            ):
                result.success = False
                result.message = 'Square exceeds turtlesim boundaries'
                goal_handle.abort()
                return result

            corners = [
                (start_x + side, start_y, 0.0),
                (start_x + side, start_y + side, math.pi / 2.0),
                (start_x, start_y + side, math.pi),
                (start_x, start_y, -math.pi / 2.0),
            ]

            for index, (x, y, yaw) in enumerate(corners):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Square drawing canceled'
                    return result

                request = GoToPose.Goal()
                request.target_pose.x = x
                request.target_pose.y = y
                request.target_pose.yaw = yaw

                goal = self.wait_for_future(
                    self.go_client.send_goal_async(request)
                )

                if goal is None or not goal.accepted:
                    result.success = False
                    result.message = (
                        f'goToPose goal {index + 1} rejected or timed out'
                    )
                    goal_handle.abort()
                    return result

                action_result = self.wait_for_future(
                    goal.get_result_async()
                )

                if (
                    action_result is None
                    or not action_result.result.success
                ):
                    result.success = False
                    result.message = f'Failed while drawing side {index + 1}'
                    goal_handle.abort()
                    return result

                if self.current_pose is not None:
                    self.get_logger().info(
                        f'Side {index + 1}: target=({x:.3f}, {y:.3f}), '
                        f'actual=({self.current_pose.x:.3f}, '
                        f'{self.current_pose.y:.3f})'
                    )

                feedback.sides_completed = index + 1
                goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = 'Square drawn using four goToPose goals'
            goal_handle.succeed()
            return result

        except Exception as exc:
            self.get_logger().error(f'Square execution failed: {exc}')
            result.success = False
            result.message = str(exc)
            if goal_handle.is_active:
                goal_handle.abort()
            return result

        finally:
            # Restore the original controller speed.
            if speed_changed and original_speed is not None:
                if not self.set_linear_speed(original_speed):
                    self.get_logger().error(
                        'Failed to restore original linear speed'
                    )

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SquareServer()

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
