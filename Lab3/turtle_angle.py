#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller')
        
        # Target configuration 
        self.goal_x = 8.0
        self.goal_y = 8.0
        
        # PID Controller
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.control_loop)
        
        # Tuned parameters
        self.kp_linear = 0.5  # Linear velocity gain
        self.kp_angular = 3.0  # Angular velocity gain
        self.tolerance = 0.1   # Arrival margin (10cm)

    def control_loop(self, msg):
        """Main control loop"""
        # 1. Calculate metrics
        dx = self.goal_x - msg.x
        dy = self.goal_y - msg.y
        distance = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        
        # 2. Log metrics
        rospy.loginfo(f"DTG: {distance:.2f}m | ATG: {math.degrees(angle):.2f}°")
        
        # 3. Publish commands
        cmd = Twist()
        if distance > self.tolerance:
            angle_error = angle - msg.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            if abs(angle_error) > 0.2:  # Turning phase
                cmd.angular.z = self.kp_angular * angle_error
            else:  # Moving forward phase
                cmd.linear.x = min(self.kp_linear * distance, 1.0)
                cmd.angular.z = 0.3 * angle_error  # Smooth correction
            
            self.vel_pub.publish(cmd)

if __name__ == '__main__':
    controller = TurtleController()
    rospy.spin()
