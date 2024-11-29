import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.subscription
        self.cmd = Twist()
        self.timer = self.create_timer(0.1, self.move_robot)
        self.obstacle_detected = False

    def lidar_callback(self, msg):
        # Analyseer de scan-data en bepaal of er een obstakel is
        front_distance = min(min(msg.ranges[0:30]), min(msg.ranges[-30:]))
        if front_distance < 0.5:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def move_robot(self):
        if self.obstacle_detected:
            # Draai in een willekeurige richting als er een obstakel is
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = random.choice([-1.0, 1.0])
        else:
            # Beweeg vooruit als er geen obstakel is
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0

        # Publiceer de beweging
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
