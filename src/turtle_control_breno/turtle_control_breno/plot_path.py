import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from threading import Thread
import time

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.x_data = []
        self.y_data = []
        self.plot_thread = Thread(target=self.plot_path)
        self.plot_thread.start()

    def odom_callback(self, msg):
        # Extract position
        position = msg.pose.pose.position
        self.x_data.append(position.x)
        self.y_data.append(position.y)

    def plot_path(self):
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()
        ax.set_title("2D Path Visualization")
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        
        while rclpy.ok():
            if self.x_data and self.y_data:
                ax.clear()
                ax.plot(self.x_data, self.y_data, label="Path")
                ax.legend()
                plt.pause(0.1)  # Update plot every 100 ms
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Keep the final plot open

if __name__ == '__main__':
    main()
