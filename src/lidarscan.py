#!/usr/bin/env python2.7
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

global speed
speed = Twist()
x = 0
y = 0
theta = 0
align = False

def get_coordinates(msg):
	found = False
	storage = []
	for (index, dist) in enumerate(msg.ranges):
		if not found and dist<25:
			found = True
			storage.append(index)
		elif found  and dist>25:
			storage.append(index-1)
			break
	n = (storage[0]+storage[1])//2
	dist = msg.ranges[n]
	goal_theta = (0.25*n*np.pi)/180 - np.pi/2
	goal_x = dist*np.sin(goal_theta) - 0.08
	goal_y = dist*np.cos(goal_theta) - 0.2
	#goal_theta = np.arctan(goal_y/goal_x)
	c = [goal_x,goal_y,goal_theta]
	return c

def pos(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def action(msg):
	global align
	coords = get_coordinates(msg)
	# rospy.loginfo(f"Found object at coordinates x: {coords[0]}, y: {coords[1]}. Now, moving")
	rospy.loginfo("Found object at coordinates x: %f" % coords[0])
	rospy.loginfo("Found object at coordinates y: %f" % coords[1])
	rospy.loginfo("Found object at theta: %f" % coords[2])
	odom = rospy.Subscriber("/mobile_base_controller/odom", Odometry, pos)
	vel_left = rospy.Publisher("/front_left_controller/command", Float64, queue_size = 1)
	vel_right = rospy.Publisher("/front_right_controller/command", Float64, queue_size = 1)
	pub_move = rospy.Publisher('/rear_drive_controller/command', Float64, queue_size=1)

	r = rospy.Rate(10)

	goal = Point()
	goal.x = coords[0]
	goal.y = coords[1]
	speed = coords[2]
	vel_left.publish(speed) 
	vel_right.publish(-speed)
	pub_move.publish(0)

	while not rospy.is_shutdown():
		#rospy.loginfo("My current pose: %f" % theta)
		if msg.ranges[359] > 1.5:
			vel_left.publish(-2) 
			vel_right.publish(-2) 
			pub_move.publish(-2)
			rospy.loginfo("Moving towards object")
		else:
			vel_left.publish(0) 
			vel_right.publish(0) 
			pub_move.publish(0)
			rospy.loginfo("Object reached. Initiate arm")
			break
	r.sleep()

if __name__=='__main__':
	rospy.init_node('vel_pub')
	sub = rospy.Subscriber('/laser/scan', LaserScan, action)
	rospy.spin()
