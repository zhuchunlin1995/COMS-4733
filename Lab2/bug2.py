#!/usr/bin/env python
import rospy
import numpy
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# For checking equality
def eq(aTuple, bTuple):
	return abs(aTuple[0] - bTuple[0]) < 0.8 and abs(aTuple[1] - bTuple[1]) < 0.8 and abs(aTuple[2] - bTuple[2]) < 0.8

# To receive laser ranges
def scan_callback(msg):
	global g_range_ahead
	global g_range_right
	g_range_ahead = msg.ranges[len(msg.ranges)//2]
	g_range_right = msg.ranges[0]

# To get current pos and orientation
def odom_callback(msg):
	global pos
	global orient
	position = msg.pose.pose.position
	orientation = msg.pose.pose.orientation
	pos = (position.x, position.y, position.z)
	orient = (orientation.x, orientation.y, orientation.z)

# Moving forward
def translate(linear):
    	linear_speed = 0.2
    	linear_duration = abs(linear / linear_speed)
    	if (linear > 0):
    		move_cmd = Twist()
    		move_cmd.linear.x = linear_speed
    	else:
    		move_cmd = Twist()
    		move_cmd.linear.x = -linear_speed
    	ticks = int(linear_duration * 50)
    	for t in range(ticks):
    		cmd_vel_pub.publish(move_cmd)
    		rate.sleep()

    	#stop the robot
    	move_cmd = Twist()
    	cmd_vel_pub.publish(move_cmd)

# Rotate left or right
def rotate(angular):
	angular_speed = 1.0
	angular_duration = abs(angular/ angular_speed)
	if (angular > 0):
		move_cmd = Twist()
		move_cmd.angular.z = angular_speed
	else:
		move_cmd = Twist()
		move_cmd.angular.z = -angular_speed
	ticks = int(angular_duration * 50)
	for t in range(ticks):
		cmd_vel_pub.publish(move_cmd)
		rate.sleep()
	# stop the robot 	
	move_cmd = Twist()
	cmd_vel_pub.publish(move_cmd)



# Init values
g_range_ahead = float('nan')
g_range_right = float('nan')
pos = (0, 0, 0)
orient = (0, 0, 0)

# Subscribe and publish topics
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
rospy.init_node('bug')
rate = rospy.Rate(50)

# Init values
hp = []
followM = True
# Whether translate of rotate
distance_ahead = 1
# Max distance to obstacle when following it
max_distance_obstacle = 2
# Min distance to obstacle when following it
min_distance_obstacle = 0.8
# Goal reaching
goal = False
# Adjusting orientation when hitting obstacal
adjust_obs = False
adjust_M = False

linear = 0.3
left = 0.4 #left
right = -0.15
small_angle = 0.04
count = 0


# The main loop, end when terminate
while (not goal):
	# When following M-line
	while followM:
		# Check goal
		if eq(pos, (10,0,0)):
			print ("Goal reached.")
			rospy.loginfo("Goal reached.")
			goal = True
			break

		# When turing from obstacle to m-line, adjust orientation
		while adjust_M:
			rotate(left)
			if abs(orient[0]) < 0.1 and abs(orient[1]) < 0.1 and abs(orient[2]) < 0.1:
				adjust_M = False

		# Keep in correct direction
		if orient[2] < 0:
			rotate(small_angle)
		else:
			rotate(-small_angle)
		translate(linear)

		# When obstacle encountered
		if (g_range_ahead < distance_ahead):
			followM = False
			hp.append([pos, False])
			adjust_obs = True
			count = 0

	# When following obstacle
	while not followM:
		# Check goal
		if eq(pos, (10,0,0)):
			print ("Goal reached.")
			rospy.loginfo("Goal reached.")
			goal = True
			break

		# When turing from m-line to obstacle, adjust orientation
		while adjust_obs:
			rotate(left)
			if (math.isnan(g_range_right) or g_range_right > max_distance_obstacle):
				adjust_obs = False

		# Following
		translate(linear)
		while (math.isnan(g_range_right) or g_range_right > max_distance_obstacle):
			rotate(right)
		
		while g_range_right < min_distance_obstacle:
			rotate(left)

		if g_range_right > g_range_ahead:
			adjust_obs = True
			
		# Check m-line hitting
		count += 1
		if count > 5:
			if (0 <= pos[0] <= 10 and abs(pos[1] - 0) < 0.5 and abs(pos[2] - 0) < 0.5):
				newHP = True
				h = hp[0]
				for i in range(len(hp)):
					if eq(hp[i][0], pos):
						newHP = False
						hp[i][1] = True
						h = hp[i]

				# If all hit point passed at least once, no solution
				allT = True
				for i in hp:
					if i[1] == False:
						allT = False

				if count > 5 and newHP:
					hp.append([pos, False])
					adjust_M = True
					followM = True

				
				elif (count > 15 and allT == True):
					print ("No solution.")
					rospy.loginfo("No solution.")
					goal = True
				break


