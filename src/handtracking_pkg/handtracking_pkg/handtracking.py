import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import cv2
import mediapipe as mp
import math
import random

class HandControl(Node):
    def __init__(self):
        super().__init__('hand_control')
        
        # Publishers and subscribers
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Initialize MediaPipe Hand tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        
        # Control states
        self.is_moving = False
        self.cmd = Twist()
        self.base_speed = 0.2
        
        # LIDAR data
        self.laser_forward = float('inf')
        self.laser_left = float('inf')
        self.laser_right = float('inf')
        self.safety_distance = 0.5
        self.turning = False
        
    def lidar_callback(self, msg):
        # Get forward distance (average of central readings)
        self.laser_forward = min(min(msg.ranges[0:20]), min(msg.ranges[340:359]))
        # Get left and right distances
        self.laser_left = min(msg.ranges[0:90])
        self.laser_right = min(msg.ranges[270:340])
        
    def calculate_finger_distance(self, hand_landmarks):
        # Calculate distance between index and thumb tip
        index_tip = hand_landmarks.landmark[8]  # Index finger tip
        thumb_tip = hand_landmarks.landmark[4]  # Thumb tip
        
        # Use 3D distance calculation including z-coordinate
        distance = math.sqrt(
            (index_tip.x - thumb_tip.x)**2 + 
            (index_tip.y - thumb_tip.y)**2 +
            (index_tip.z - thumb_tip.z)**2
        )
        return distance * 2.0  # Amplify the distance for better control
    
    def detect_hand_gesture(self, hand_landmarks):
        # Detect open/closed hand
        finger_tips = [8, 12, 16, 20]  # Index, middle, ring, pinky tips
        finger_count = 0
        
        for tip in finger_tips:
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
                finger_count += 1
                
        # Check thumb separately
        if hand_landmarks.landmark[4].x > hand_landmarks.landmark[3].x:
            finger_count += 1
            
        return finger_count
    
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
            # Keep turning until we have clear path
            if self.laser_forward > self.safety_distance * 1.5:
                self.turning = False
            return True
        return False
    
    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        # Default to stopping if no hand detected
        current_speed = 0.0
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS
                )
                
                finger_count = self.detect_hand_gesture(hand_landmarks)
                
                # Handle gestures
                if finger_count == 2:  # Two-finger speed control
                    distance = self.calculate_finger_distance(hand_landmarks)
                    self.base_speed = max(0.1, min(distance, 0.5))  # Limit speed between 0.1 and 0.5
                    current_speed = self.base_speed
                    self.is_moving = True  # Ensure movement when controlling speed
                        
                elif finger_count >= 4:  # Open hand
                    current_speed = self.base_speed
                    self.is_moving = True
                else:  # Closed hand
                    current_speed = 0.0
                    self.is_moving = False
        
        # Set forward speed if not avoiding obstacles
        if not self.avoid_obstacles():  # Removed self.is_moving check here
            self.cmd.linear.x = current_speed if self.is_moving else 0.0
            self.cmd.angular.z = 0.0
            
        # Display status on frame
        cv2.putText(frame, f"Speed: {self.cmd.linear.x:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Forward dist: {self.laser_forward:.2f}", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        self.publisher_.publish(self.cmd)
        cv2.imshow('Hand Control', frame)
        cv2.waitKey(1)
        
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    hand_control = HandControl()
    
    try:
        rclpy.spin(hand_control)
    except KeyboardInterrupt:
        pass
    finally:
        hand_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()