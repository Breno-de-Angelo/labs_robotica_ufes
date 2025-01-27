import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_interfaces.action import TurtleGoal
from rclpy.action import ActionClient

GOAL_POSES = [Pose() for _ in range(4)]
GOAL_POSES[0].position.x = 0.5
GOAL_POSES[0].position.y = 0.5
GOAL_POSES[0].position.z = 0.0
GOAL_POSES[1].position.x = 0.5
GOAL_POSES[1].position.y = 0.0
GOAL_POSES[1].position.z = 0.0
GOAL_POSES[2].position.x = 0.0
GOAL_POSES[2].position.y = 0.5
GOAL_POSES[2].position.z = 0.0
GOAL_POSES[3].position.x = 0.0
GOAL_POSES[3].position.y = 0.0
GOAL_POSES[3].position.z = 0.0

class TurtleClient(Node):
    current_goal_index: int = 0

    def __init__(self):
        super().__init__('turtle_supervisor')
        self.get_logger().info("Initializing turtle client.")
        self._action_client = ActionClient(self, TurtleGoal, '/turtle_goal')
    
    def send_goal(self):
        self.get_logger().info(f"Sending goal {self.current_goal_index}")
        goal_msg = TurtleGoal.Goal()
        goal_msg.goal = GOAL_POSES[self.current_goal_index]
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Current Pose: {feedback_msg.feedback.pose}")        
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")
        self.current_goal_index = (self.current_goal_index + 1) % len(GOAL_POSES)
        time.sleep(3)
        self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    turtle_client = TurtleClient()

    turtle_client.send_goal()

    rclpy.spin(turtle_client)

    turtle_client.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
