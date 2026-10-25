
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from lab1_interfaces.action import GoToPose


class GoToPoseServer(Node):

    def __init__(self):
        super().__init__('go_to_pose_server')

        # Maximum movement speeds
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )

        # Allow parameters to change while running
        self.add_on_set_parameters_callback(
            self.parameters_callback
        )

        self.pose = None
        self.group = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self.group
        )

        self.action_server = ActionServer(
            self,
            GoToPose,
            '/goToPose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group
        )

        self.get_logger().info(
            'goToPose action server ready'
        )

    def parameters_callback(self, params):
        new_linear = self.max_linear_speed
        new_angular = self.max_angular_speed

        for param in params:
            if param.name == 'max_linear_speed':
                if (
                    param.type_ != param.Type.DOUBLE
                    or not math.isfinite(param.value)
                    or param.value <= 0.0
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='max_linear_speed must be a positive double'
                    )

                new_linear = float(param.value)

            elif param.name == 'max_angular_speed':
                if (
                    param.type_ != param.Type.DOUBLE
                    or not math.isfinite(param.value)
                    or param.value <= 0.0
                ):
                    return SetParametersResult(
                        successful=False,
                        reason='max_angular_speed must be a positive double'
                    )

                new_angular = float(param.value)

        self.max_linear_speed = new_linear
        self.max_angular_speed = new_angular

        self.get_logger().info(
            f'Speeds updated: linear={self.max_linear_speed}, '
            f'angular={self.max_angular_speed}'
        )

        return SetParametersResult(successful=True)

    def pose_callback(self, msg):
        self.pose = msg

    def goal_callback(self, request):
        target = request.target_pose

        if not all(math.isfinite(value) for value in (
            target.x,
            target.y,
            target.yaw
        )):
            return GoalResponse.REJECT

        if not (0.5 <= target.x <= 10.5):
            return GoalResponse.REJECT

        if not (0.5 <= target.y <= 10.5):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def execute_callback(self, goal_handle: ServerGoalHandle):
        target = goal_handle.request.target_pose

        result = GoToPose.Result()
        feedback = GoToPose.Feedback()
        twist = Twist()

        # Tolerances for position and final orientation
        position_tolerance = 0.02
        yaw_tolerance = 0.03

        try:
            # Wait until the first pose message arrives
            while rclpy.ok() and self.pose is None:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled'
                    return result

                time.sleep(0.05)

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled'
                    return result

                if self.pose is None:
                    time.sleep(0.05)
                    continue

                dx = target.x - self.pose.x
                dy = target.y - self.pose.y
                distance = math.hypot(dx, dy)

                heading = math.atan2(dy, dx)
                heading_error = self.normalize_angle(
                    heading - self.pose.theta
                )

                yaw_error = self.normalize_angle(
                    target.yaw - self.pose.theta
                )

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                if distance > position_tolerance:
                    # Rotate toward the target before moving
                    if abs(heading_error) > 0.12:
                        twist.angular.z = max(
                            -self.max_angular_speed,
                            min(
                                self.max_angular_speed,
                                4.0 * heading_error
                            )
                        )
                    else:
                        # Move forward while correcting heading
                        twist.linear.x = min(
                            self.max_linear_speed,
                            1.5 * distance
                        )

                        twist.angular.z = max(
                            -self.max_angular_speed,
                            min(
                                self.max_angular_speed,
                                2.0 * heading_error
                            )
                        )

                elif abs(yaw_error) > yaw_tolerance:
                    # Rotate to the requested final orientation
                    twist.angular.z = max(
                        -self.max_angular_speed,
                        min(
                            self.max_angular_speed,
                            3.0 * yaw_error
                        )
                    )

                else:
                    break

                self.cmd_pub.publish(twist)

                feedback.distance_remaining = distance
                goal_handle.publish_feedback(feedback)

                time.sleep(0.05)

            if not rclpy.ok():
                result.success = False
                result.message = 'ROS shutdown before reaching target'
                goal_handle.abort()
                return result

        except Exception as exc:
            self.get_logger().error(
                f'Action execution failed: {exc}'
            )
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
            return result

        finally:
            # Always stop the turtle when execution ends
            self.cmd_pub.publish(Twist())

        goal_handle.succeed()
        result.success = True
        result.message = 'Target pose reached'
        return result


def main(args=None):
    rclpy.init(args=args)

    node = GoToPoseServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
