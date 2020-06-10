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
			rospy.loginfo("reached destination")
			break

	
	vel_msg.linear.x = 0
	velocity_publisher.publish(vel_msg)







if __name__ == '__main__':
	try:
		# declare robot_cleaner node
		rospy.init_node('robot_cleaner', anonymous=True)

		# declare the publisher
		velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)

		pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, poseCallback)

		time.sleep(2);

		move(1.0, 5.0, True)
		move(1.0, 5.0, False)
	
	except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


