#!/usr/bin/env python3
import rospy
from turtlesim.srv import TeleportAbsolute

def setup_turtle():
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        # Create a service proxy for teleportation
        teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        # Teleport turtle1 to starting position (x=2.0, y=2.0) facing right (θ=0.0)
        teleport(2.0, 2.0, 0.0)  
        # Log success message
        rospy.loginfo("Turtle in the starting position (2.0, 2.0)")
    except rospy.ServiceException as e:
        # Log error if teleportation fails
        rospy.logerr(f"Error al teleportar: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('turtle_spawner')
    setup_turtle()