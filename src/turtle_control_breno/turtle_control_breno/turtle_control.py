import math
from custom_interfaces.action import TurtleGoal
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from typing import Optional
from rclpy.action import ActionServer
from rclpy.parameter_event_handler import ParameterEventHandler
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
import rclpy.time

class TurtleController(Node):
    cmd_vel_pub: Publisher

    v_max: float
    k_omega: float
    epsilon: float

    turtle_pose: Optional[Pose] = None
    goal_pose: Optional[Pose] = None

    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Initializing turtle controller.")

        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('k_omega', 0.2)
        self.declare_parameter('epsilon', 0.01)
        self.handler = ParameterEventHandler(self)

        self.v_max = self.get_parameter('v_max').value
        self.k_omega = self.get_parameter('k_omega').value
        self.epsilon = self.get_parameter('epsilon').value

        self.handler.add_parameter_callback(
            parameter_name="v_max",
            node_name="turtle_controller",
            callback=self.set_v_max,
        )
        self.handler.add_parameter_callback(
            parameter_name="k_omega",
            node_name="turtle_controller",
            callback=self.set_k_omega,
        )
        self.handler.add_parameter_callback(
            parameter_name="epsilon",
            node_name="turtle_controller",
            callback=self.set_epsilon,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.goal_action = ActionServer(self, TurtleGoal, "/turtle_goal", self.goal_callback)

    def set_v_max(self, value: Parameter):
        self.v_max = value.value.double_value
        self.get_logger().info(f"Updated v_max: {value}")
    
    def set_k_omega(self, value: Parameter):
        self.get_logger().info(f"Updated k_omega: {value}")
        self.k_omega = value.value.double_value

    def set_epsilon(self, value: Parameter):
        self.get_logger().info(f"Updated epsilon: {value}")
        self.epsilon = value.value.double_value

    def odom_callback(self, msg: Odometry):
        self.turtle_pose = msg.pose.pose

    def goal_callback(self, request):
        self.get_logger().info(f"Received goal: {request.request.goal}")
        self.goal_pose = request.request.goal

        if not self.turtle_pose:
            self.get_logger().warn("Turtle pose not available")
            request.abort()
            result = TurtleGoal.Result()
            result.success = False
            return result

        rate = self.create_rate(5)
        while True:
            feedback_msg = TurtleGoal.Feedback()
            feedback_msg.pose = self.turtle_pose
            request.publish_feedback(feedback_msg)

            rho2 = (self.goal_pose.position.x - self.turtle_pose.position.x) ** 2 + (self.goal_pose.position.y - self.turtle_pose.position.y) ** 2
            if rho2 < self.epsilon ** 2:
                self.get_logger().info(f"Goal reached, distance = {math.sqrt(rho2)}")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                request.succeed()
                result = TurtleGoal.Result()
                result.success = True
                return result

            current_angle = math.atan2(2 * self.turtle_pose.orientation.z * self.turtle_pose.orientation.w,
                                       1 - 2 * self.turtle_pose.orientation.z ** 2)
            self.get_logger().info(f"Current angle: {current_angle}")
            if current_angle < 0:
                normalized_current_angle = 2 * math.pi + current_angle
            else:
                normalized_current_angle = current_angle
            self.get_logger().info(f"Normalized current angle: {normalized_current_angle}")

            goal_angle = math.atan2(self.goal_pose.position.y - self.turtle_pose.position.y,
                                    self.goal_pose.position.x - self.turtle_pose.position.x)
            self.get_logger().info(f"Goal angle: {goal_angle}")
            if goal_angle < 0:
                normalized_goal_angle = 2 * math.pi + goal_angle
            else:
                normalized_goal_angle = goal_angle
            self.get_logger().info(f"Normalized goal angle: {normalized_goal_angle}")

            alpha = normalized_goal_angle - normalized_current_angle

            twist_msg = Twist()
            twist_msg.linear.x = self.v_max * math.tanh(math.sqrt(rho2))
            twist_msg.angular.z = self.k_omega * alpha

            self.cmd_vel_pub.publish(twist_msg)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    executor = MultiThreadedExecutor()
    rclpy.spin(turtle_controller, executor=executor)

    turtle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
