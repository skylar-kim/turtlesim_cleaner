#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import sys

turtlesim_pose = Pose()
turtlesim_pose_x = 0
turtlesim_pose_y = 0
turtlesim_pose_theta = 0

def move(speed, distance, isForward):
	vel_msg = Twist()

	# set the linear velocity (positive if forward, negative if backward)
	if (isForward):
		vel_msg.linear.x = abs(speed)
	else:
		vel_msg.linear.x = -abs(speed)

	# y and z linear vel is 0 because 2D space
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0

	# set the angular velocity to 0 bc only moving straight
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	# t0 is current time
	t0 = rospy.Time.now().to_sec()
	current_distance = 0

	loop_rate = rospy.Rate(10)

	while True:
		# publish the velocity message
		velocity_publisher.publish(vel_msg)
		# measure the current time
		t1 = rospy.Time.now().to_sec()
		# estimate the current distance moved by robot
		current_distance = speed * (t1-t0)

		loop_rate.sleep()

		if (current_distance > distance):
                    rospy.loginfo("reached distance")
                    break

	
	vel_msg.linear.x = 0
	velocity_publisher.publish(vel_msg)

def rotate(angular_speed, relative_angle, clockwise):
	# clockwise = negative angle
	# counter clockwise = positive angle
	vel_msg = Twist()

	current_angle = 0.0

	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0

	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	if (clockwise):
		vel_msg.angular.z = -abs(angular_speed)
	else:
		vel_msg.angular.z = abs(angular_speed)

	# the time before entering into the loop
	t0 = rospy.Time.now().to_sec()

	# loop will execute 10 times per second
	loop_rate = rospy.Rate(10)

	# publish the velocity message
	while True:
		velocity_publisher.publish(vel_msg)

		# the current time
		t1 = rospy.Time.now().to_sec()

		current_angle = angular_speed * (t1-t0)

		loop_rate.sleep()

		if (current_angle > relative_angle):
			rospy.loginfo("reached destination angle")
			break

	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)

def degrees2radians(angle_in_degrees):
	return angle_in_degrees * (math.pi / 180.0)

def setDesiredOrientation(desired_angle_radians):
	global turtlesim_pose_theta

	# to find the relative angle, subtract the turtlesim_pose_theta
	# (the current orientation of the robot) NOT turtlesim_pose.theta 
	# from the desired_angle_radians
	# how did we get the theta of the turtlesim pose? we created a global variable...
	relative_angle_radians = desired_angle_radians - turtlesim_pose_theta
	clockwise = True if (relative_angle_radians < 0) else False

	rotate(degrees2radians(10), abs(relative_angle_radians), clockwise)


def poseCallback(pose_message):
	global turtlesim_pose_x
	global turtlesim_pose_y
	global turtlesim_pose_theta

	# this is setting the turtlesim_pose to the last updated message
	# from the callback function
	turtlesim_pose_x = pose_message.x
	turtlesim_pose_y = pose_message.y
	turtlesim_pose_theta = pose_message.theta

def getDistance(x1,y1,x2,y2):
	return math.sqrt(pow((x1-x2),2) + pow((y1-y2),2))

def moveGoal(goal_pose, distance_tolerance):
	global turtlesim_pose_x
	global turtlesim_pose_y
	global turtlesim_pose_theta

	vel_msg = Twist()

	loop_rate = rospy.Rate(10)

	while (getDistance(turtlesim_pose_x, turtlesim_pose_y, goal_pose.x, goal_pose.y) > distance_tolerance):
		# Proportional Controller
		vel_msg.linear.x = 1.5 * getDistance(turtlesim_pose_x, turtlesim_pose_y, goal_pose.x, goal_pose.y)
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		# define angular velocity in the z-axis
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 4 * (math.atan2(goal_pose.y - turtlesim_pose_y, goal_pose.x - turtlesim_pose_x)-turtlesim_pose_theta)

		# publish the message...will this work???
		velocity_publisher.publish(vel_msg)

		loop_rate.sleep()

	rospy.loginfo("End moveGoal")

	# force stop the robot
	vel_msg.linear.x = 0
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)

def gridClean():
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0
 	
    

    moveGoal(desired_pose, 0.01)
    setDesiredOrientation(0)
    
 
    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)

    move(2.0, 9.0, True)
    rotate(degrees2radians(20), degrees2radians(90), False)

    for i in range(0,6):
    	rospy.loginfo("starting loop: %s", i)
    	move(2.0, 1.0, True)
    	rotate(degrees2radians(20), degrees2radians(90), False)

    	move(2.0, 9.0, True)
    	rotate(degrees2radians(30), degrees2radians(90), True)

    	move(2.0, 1.0, True)
    	rotate(degrees2radians(30), degrees2radians(90), True)

    	move(2.0, 9.0, True)
    	rotate(degrees2radians(30), degrees2radians(90), False)

    	rospy.loginfo("ending loop: %s", i)

    
    # setDesiredOrientation(0)
    

    # move(2.0, 1.0, True)
    # rotate(degrees2radians(30), degrees2radians(90), False)

    # move(2.0, 9.0, True)
    # rotate(degrees2radians(30), degrees2radians(90), True)

    # move(2.0, 1.0, True)
    # rotate(degrees2radians(30), degrees2radians(90), True)

    # move(2.0, 9.0, True)
    # rotate(degrees2radians(30), degrees2radians(90), False)

    pass

def spiralClean():
	global turtlesim_pose_x
	global turtlesim_pose_y

	vel_msg = Twist()
	count = 0

	constant_speed = 4
	vk = 1
	wk = 2
	rk = 0.5

	loop_rate = rospy.Rate(1)

	# the idea to make the robot go in a spiral:
	# increase the linear velocity so that the radius will also increase
	# if the lin vel is constant, then we have a circle
	# if the lin vel is increasing, then we have a spiral
	while ((turtlesim_pose_x < 10.5) and (turtlesim_pose_y < 10.5)):
		# depending on the number added to rk, how tight the spiral is, will be affected
		rk = rk + 0.5
		vel_msg.linear.x = rk
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = constant_speed

		rospy.loginfo("vel_msg.linear.x = %s", vel_msg.linear.x)
		rospy.loginfo("vel_msg.angular.z = %s", vel_msg.angular.z)

		velocity_publisher.publish(vel_msg)

		loop_rate.sleep()

		rospy.loginfo("rk: %s, vk: %s, wk: %s", rk, vk, wk)

	# force the robot to stop
	vel_msg.linear.x = 0
	velocity_publisher.publish(vel_msg)







if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback) 
        time.sleep(2)
        
        # menu for following inputs
        # 1 is spiral cleaning
        # 2 is grid cleaning
        print ("What kind of cleaning would you like?")
        print ("\nType 1 for spiral cleaning")
        print ("\nType 2 for grid cleaning")

        clean = input("\nPlease enter a number: ")

        if (clean == 1):
        	spiralClean()
        elif (clean == 2):
        	gridClean()
        


        # move(1.0, 5.0, False)
        # rotate(degrees2radians(30), degrees2radians(90), True)

        # setDesiredOrientation(degrees2radians(90))
        # setDesiredOrientation(degrees2radians(-90))
        # setDesiredOrientation(degrees2radians(-90))

        # turtlesim_pose_x = 1
        # turtlesim_pose_y = 1
        # turtlesim_pose_theta = 0

        # moveGoal(turtlesim_pose, 0.01) 
        #setDesiredOrientation(math.radians(90))
        # spiralClean()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


