import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Subscriber for hand gesture commands
        self.gesture_subscription = self.create_subscription(
            Twist,
            'hand_gesture_cmd',
            self.gesture_callback,
            10
        )
        
        self.cmd = Twist()
        self.timer = self.create_timer(0.1, self.move_robot)
        self.laser_forward = float('inf')
        self.laser_left = float('inf')
        self.laser_right = float('inf')
        self.safety_distance = 0.5
        self.turning = False
        self.current_speed = 0.0  # Initialize current speed

    def lidar_callback(self, msg):
        # Get forward distance (average of central readings)
        self.laser_forward = min(min(msg.ranges[0:20]), min(msg.ranges[340:359]))
        # Get left and right distances
        self.laser_left = min(msg.ranges[30:90])
        self.laser_right = min(msg.ranges[270:330])

    def gesture_callback(self, msg):
        # Update current speed based on gesture command
        self.current_speed = msg.linear.x

    def avoid_obstacles(self):
        if self.laser_forward < self.safety_distance:
            self.turning = True
            self.cmd.linear.x = 0.0
            # Choose turn direction based on side distances
            if self.laser_left > self.laser_right:
                self.cmd.angular.z = 0.5  # Turn left
            else:
                self.cmd.angular.z = -0.5  # Turn right
            return True
        elif self.turning:
            # Keep turning until we have a clear path
            if self.laser_forward > self.safety_distance * 1.5:
                self.turning = False
            return True
        return False

    def move_robot(self):
        if not self.avoid_obstacles():
            self.cmd.linear.x = self.current_speed  # Use speed from gesture command
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.0  # Stop if avoiding obstacles

        # Publish the movement command
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    
    try:
        rclpy.spin(obstacle_avoidance)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_avoidance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()