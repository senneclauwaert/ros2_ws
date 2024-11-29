import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar_navigation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.timer_period = 0.1
        self.laser_forward = 0.0
        self.laser_frontLeft = 0.0
        self.laser_frontRight = 0.0
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]
        self.laser_frontLeft = min(msg.ranges[0:15])
        self.laser_frontRight = min(msg.ranges[345:359])

    def motion(self):
        self.get_logger().info(f"Forward: {self.laser_forward:.2f}, Left: {self.laser_frontLeft:.2f}, Right: {self.laser_frontRight:.2f}")
        
        self.cmd.linear.x = 0.2
        self.cmd.angular.z = 0.0

        if self.laser_forward < 0.5:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = random.choice([-1.0, 1.0])
        elif self.laser_frontLeft < 0.5:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = -1.0
        elif self.laser_frontRight < 0.5:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 1.0

        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
