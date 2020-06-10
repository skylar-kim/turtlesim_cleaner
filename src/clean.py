#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

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
	global turtlesim_pose

	# to find the relative angle, subtract the 
	relative_angle_radians = desired_angle_radians - turtlesim_pose.theta
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
    setDesiredOrientation(degrees2radians(0))
    
 
    # move(2.0, 9.0, True)
    # rotate(degrees2radians(20), degrees2radians(90), False)
    # move(2.0, 9.0, True)
    # rotate(degrees2radians(20), degrees2radians(90), False)
    # move(2.0, 1.0, True)
    # rotate(degrees2radians(20), degrees2radians(90), False)
    # move(2.0, 9.0, True)
    # rotate(degrees2radians(30), degrees2radians(90), True)
    # move(2.0, 1.0, True)
    # rotate(degrees2radians(30), degrees2radians(90), True)
    # move(2.0, 9.0, True)
    pass









if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        
        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback) 
        time.sleep(2)

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
        gridClean()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


