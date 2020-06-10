#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

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




if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        #position_topic = "/turtle1/pose"
        #pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        move(1.0, 5.0, False)
        rotate(degrees2radians(30), degrees2radians(90), True)
        #go_to_goal(1.0, 1.0) 
        #setDesiredOrientation(math.radians(90))
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


