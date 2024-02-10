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
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_square_driver', anonymous=True)

        # Reset turtlesim by calling the reset service
        # Not necessary when running from for launch files
        rospy.wait_for_service('/gazebo/reset_world')
        resetTurtle = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resetTurtle()

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=3)

        self.rate = rospy.Rate(50)
        
    def traverseSquare(self):
        """Move in a square."""
        
        # Get the input from the user.        
        v = rospy.get_param('~v')
        w = rospy.get_param('~w')

        vel_msg = Twist()

        linPeriod = int(10*1e9) # Time period to move linearly 
        rotPeriod = int(7.854*1e9) # Time period to rotate

        while self.blank:
            pass
         
        for i in range(4):
            t_start = time.time_ns()

            while time.time_ns() < t_start + linPeriod:
                vel_msg.linear.x = v
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                self.rate.sleep()
            
            while time.time_ns() < t_start + linPeriod + rotPeriod:
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = w

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                self.rate.sleep()

        # while self.inMotion:
        #     rospy.loginfo(self.pose.x)
        #     # Velocity in Inertial Frame
        #     dX_i = np.array([0,0]).T

        #     if self.stage == 1:
        #         dX_i = np.array([v, 0]).T #move right
            
        #     elif self.stage == 2:                
        #         dX_i = np.array([0, v]).T #move up
            
        #     elif self.stage == 3:                
        #         dX_i = np.array([-v,0]).T #move left
                
        #     elif self.stage == 4:                
        #         dX_i = np.array([0,-v]).T #move down
            
        #     # Velocity in Robot Frame
        #     dX_b = np.dot(getRotationMatrix(remapAngle(self.pose.theta)).T, dX_i)

        #     # Linear velocity in the x-axis.
        #     vel_msg.linear.x = dX_b[0]
        #     vel_msg.linear.y = dX_b[1]
        #     vel_msg.linear.z = 0

        #     # Angular velocity in the z-axis.
        #     vel_msg.angular.x = 0
        #     vel_msg.angular.y = 0
        #     vel_msg.angular.z = w

        #     # Publishing our vel_msg
        #     self.velocity_publisher.publish(vel_msg)

        #     # Publish at the desired rate.
        #     self.rate.sleep()

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
