import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Lidar(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('lidar')
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
    
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.SYSTEM_DEFAULT)
        )
        
        
        self.laser_forward = 0.0
        self.safety_distance = 0.3  # Threshold distance in meters
        
        self.cmd = Twist()
        self.timer = self.create_timer(0.1, self.motion)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[0]

    def motion(self):
        self.get_logger().info(f"Forward Distance: {self.laser_forward:.2f} m")

        if self.laser_forward > self.safety_distance:
            self.cmd.linear.x = 0.2  # Forward speed
            self.cmd.angular.z = 0.0  # No rotation
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
        
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    lidar = Lidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
