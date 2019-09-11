#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
'''
'''
Modified for HSRW Robotics
MTC 05/2019
Created for HSRW Open Day 2019
'''
# This enables TurtleBot to autonomously navigate to specified coordinates in a known map,
# wait for a specified time period and then return to Home

# TurtleBot must have bringup.launch & amcl_demo.launch running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 180 seconds to complete task (Chnage this based on map complexity)
	success = self.move_base.wait_for_result(rospy.Duration(180)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_map', anonymous=True)
        navigator = GoToPose()

        # Home (Starting position coordinates)
        position_start = {'x': -2.054, 'y' : -3.717}
        quaternion_start = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.342, 'r4' : 0.94000}
	# ASGARD (End position coordinates)
        position_end={'x':-5.920, 'y': -4.705}
        quaternion_end = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.9206, 'r4' : 0.390403}
        
        
        rospy.loginfo("Go to (%s, %s) pose", position_end['x'], position_end['y'])
        success_goal = navigator.goto(position_end, quaternion_end)

        if success_goal:
            rospy.loginfo("Hooray, reached the desired pose")
            rospy.sleep(10) # Let's wait for 10 seconds

	    rospy.loginfo("Go to (%s, %s) pose", position_start['x'], position_start['y'])
            success_start=navigator.goto(position_start, quaternion_start)
            if success_start:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

