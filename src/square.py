#!/usr/bin/env python3

import rospy

from math import pi, radians

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import time

# Remaps (-pi/2, pi/2) to (0, 2pi)
def remapAngle(angle):
    return round((angle + (2*pi)) % (2*pi), 4)

class TurtleBot:

    def __init__(self):
        # Creates a node 'turtlebot_circle_driver'. Using anonymous=True makes it unique.
        rospy.init_node('turtlebot_square_driver', anonymous=True)

        # Reset turtlesim by calling the reset service
        # Not necessary when running from for launch files
        rospy.wait_for_service('/gazebo/reset_world')
        resetTurtle = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resetTurtle()

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

        self.rate = rospy.Rate(50)
        
    def traverseSquare(self):
        """Move in a square."""
        
        # Get the input from the user.        
        v = rospy.get_param('~v')
        w = rospy.get_param('~w')

        vel_msg = Twist()

        linPeriod = (2/v) # Time period to move linearly 
        rotPeriod = (pi/(2*w)) # Time period to rotate
         
        for i in range(4):
            # Using inbuilt function get_time that listens to /clock topic
            t_start = rospy.get_time()

            while rospy.get_time() < t_start + linPeriod:
                vel_msg.linear.x = v
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0      

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            t_start = rospy.get_time()

            while rospy.get_time() < t_start + rotPeriod:
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = w
                
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

        # Stop motion
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        # rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.traverseSquare()

    except rospy.ROSInterruptException:
        pass
