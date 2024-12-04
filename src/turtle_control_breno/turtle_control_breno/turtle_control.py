import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose

class TurtleController(Node):
    cmd_vel_pub: Publisher
    v_max: float
    k_omega: float

    turle_pose: Pose | None = None
    goal_pose: Pose2D | None = None

    def __init__(self):
        super().__init__('turtle_controller')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_publisher(self):
        self.cmd_vel_pub = self.create_publisher(Twist, "/breno/turtle1/cmd_vel", 10)
        self.cmd_vel_timer = self.create_timer(0.1, self.pub_callback)

    def init_subscribers(self):
        self.pose_sub = self.create_subscription(Pose, "/breno/turtle1/pose", self.pose_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, "/goal", self.goal_callback, 10)

    def init_variables(self):
        self.v_max = 1.0
        self.k_omega = 1.0

    def pose_callback(self, msg: Pose):
        self.turle_pose = msg

    def goal_callback(self, msg: Pose2D):
        self.goal_pose = msg

    def pub_callback(self):
        if self.goal_pose and self.turle_pose:
            rho = math.sqrt((self.goal_pose.x - self.turle_pose.x) ** 2 + (self.goal_pose.y - self.turle_pose.y) ** 2)
            if rho < 0.1:
                return

            alpha = math.atan2(self.goal_pose.y - self.turle_pose.y, self.goal_pose.x - self.turle_pose.x) - self.turle_pose.theta

            self.get_logger().info(f"Rho: {rho}")
            self.get_logger().info(f"Alpha: {alpha}")

            twist_msg = Twist()
            twist_msg.linear.x = self.v_max * math.tanh(rho)
            twist_msg.angular.z = self.k_omega * alpha

            self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
