#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
license removed for brevity
'''

'''
Modified by/for HSRW Robotics
Mamen Thomas Chembakasseril 09/2019
'''
# An example of TurtleBot 2 drawing a 0.6 meter square.
# Written for indigo

import rospy
from geometry_msgs.msg import Twist
from math import radians

class GoingSquare():
    def __init__(self):
        # initiliaze
        rospy.init_node('gosquare', anonymous=True)

        # What to do when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
     
	# 5 HZ
        r = rospy.Rate(5);

	# create two different Twist() variables.  (1) One for moving forward.  (2) One for turning 45 degrees.

        # (1) let's go forward at 0.3 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
	# by default angular.z is 0 so setting this isn't required

        # (2) let's turn at 45 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(45); #45 deg/s in radians/s

	# to keep drawing squares.
	count = 0
        while not rospy.is_shutdown():
	    # go forward 0.6 m (2 seconds * 0.3 m / seconds)
	    rospy.loginfo("Going Straight")
            for x in range(0,10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
	    # turn 90 degrees
	    rospy.loginfo("Turning")
            for x in range(0,10):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()            
	    count = count + 1
	    if(count == 4): 
                count = 0
	    if(count == 0): 
                rospy.loginfo("TurtleBot should be close to the original starting position (but it's probably way off)")
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Going in Squares")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        GoingSquare()
    except:
        rospy.loginfo("Node terminated.")


