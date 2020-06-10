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
                    rospy.loginfo("reached")
                    break

	
	vel_msg.linear.x = 0
	velocity_publisher.publish(vel_msg)







if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        #position_topic = "/turtle1/pose"
        #pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        move(1.0, 2.0, False)
        #rotate(30, 90, True)
        #go_to_goal(1.0, 1.0) 
        #setDesiredOrientation(math.radians(90))
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


