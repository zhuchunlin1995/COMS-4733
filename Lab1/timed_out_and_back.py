#!/usr/bin/env python

""" timed_out_and_back.py - Version 1.2 2014-12-14
    A basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # How fast will we update the robot's movement?
        self.rate = 50
        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)

        self.command()
        
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def command(self):
    	var = raw_input("Enter T(translation), R(rotation) or Q(quit):\n")
    	while True:
    		if var != "T" and var != "R" and var != "Q":
    			var = raw_input("Wrong Input! Enter T(translation), R(rotation) or Q(quit):\n")
    		if var == "T":
    			self.translate()
    		if var == "R":
    			self.rotate()
    		if var == "Q":
    			self.quit()
    			break
    		var = raw_input("Enter T(translation), R(rotation) or Q(quit)\n")

    def translate(self):
    	linear_speed = 0.2
    	goal_distance = float(raw_input("Enter the distance:\n"))
    	linear_duration = abs(goal_distance / linear_speed)
    	if (goal_distance > 0):
    		move_cmd = Twist()
    		move_cmd.linear.x = linear_speed
    	else:
    		move_cmd = Twist()
    		move_cmd.linear.x = -linear_speed
    	ticks = int(linear_duration * self.rate)
    	for t in range(ticks):
    		self.cmd_vel.publish(move_cmd)
    		self.r.sleep()

    	#stop the robot
    	move_cmd = Twist()
    	self.cmd_vel.publish(move_cmd)

    def rotate(self):
    	angular_speed = 1.0
    	goal_angle = float(raw_input("Enter the angle:\n"))
    	angular_duration = abs(goal_angle/ angular_speed)
    	if (goal_angle > 0):
    		move_cmd = Twist()
    		move_cmd.angular.z = angular_speed
    	else:
    		move_cmd = Twist()
    		move_cmd.angular.z = -angular_speed
    	ticks = int(angular_duration * self.rate)
    	for t in range(ticks):
    		self.cmd_vel.publish(move_cmd)
    		self.r.sleep()
    	# stop the robot 	
    	move_cmd = Twist()
    	self.cmd_vel.publish(move_cmd)
    
    def quit(self):
    	print("Goodbye!")

 
if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
