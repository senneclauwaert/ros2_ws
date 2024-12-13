import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp
import math

class CameraDetection(Node):
    def __init__(self):
        super().__init__('camera_detection')
        
        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, 'hand_gesture_cmd', 10)
        
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
        self.base_speed = 0.0
        
        # Create a timer for processing frames
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
            
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        # Create a Twist message
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
                finger_count = self.detect_hand_gesture(hand_landmarks)
                
                # Handle gestures
                if finger_count == 2:  # Two-finger speed control
                    distance = self.calculate_finger_distance(hand_landmarks)
                    self.base_speed = max(0.1, min(distance, 0.5))  # Limit speed between 0.1 and 0.5
                    cmd_msg.linear.x = self.base_speed
                    self.is_moving = True
                elif finger_count >= 4:  # Open hand
                    cmd_msg.linear.x = self.base_speed
                    self.is_moving = True
                else:  # Closed hand
                    cmd_msg.linear.x = 0.0
                    self.is_moving = False
        
        self.publisher_.publish(cmd_msg)

        # Display speed and distance on the camera feed
        cv2.putText(frame, f"Speed: {cmd_msg.linear.x:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Moving: {'Yes' if self.is_moving else 'No'}", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Camera Detection', frame)
        cv2.waitKey(1)

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

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    camera_detection = CameraDetection()
    
    try:
        rclpy.spin(camera_detection)
    except KeyboardInterrupt:
        pass
    finally:
        camera_detection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()