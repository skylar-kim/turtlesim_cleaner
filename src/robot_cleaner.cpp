#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

// global variable declarations
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;


// methods
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
double setDesiredOrientation(double desired_angle_radians);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;

	double speed;
	double distance;
	bool isForward;

	double angular_speed;
	double angle;
	bool clockwise;

	// declare the publisher 
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	// the pose subscriber is subscribed to the turtle1/pose topic and call the
	// poseCallback function
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

	ROS_INFO("\n\n\n*********START TESTING*********\n");

	// cout << "Enter your speed: ";
	// cin >> speed;
	// cout << "Enter your distance: ";
	// cin >> distance;
	// cout << "Forward?: ";
	// cin >> isForward;
	// move(speed, distance, isForward);

	// cout << "Enter your angular speed (degree/sec): ";
	// cin >> angular_speed;
	// cout << "Enter desired angle (degrees): ";
	// cin >> angle;
	// cout << "Going clockwise? (1 for yes, 0 for no): ";
	// cin >> clockwise;
	// rotate(degrees2radians(angular_speed), degrees2radians(angle), clockwise);

	setDesiredOrientation(degrees2radians(120));
	ros::Rate loop_rate(0.5);
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(-60));
	loop_rate.sleep();
	setDesiredOrientation(degrees2radians(0));

	ros::spin();
	
	return 0;
}

/* 
	makes the robot move with a certain linear velocity for a 
	certain distance in a forward or backward straight direction
*/

void move(double speed, double distance, bool isForward)
{
	// distance = speed * time
	geometry_msgs::Twist vel_msg;

	// set the linear velocity (positive if forward, negative if backward)
	if (isForward) {
		vel_msg.linear.x = abs(speed);
	}
	else {
		vel_msg.linear.x = -abs(speed);
	}
	// y and z linear vel is 0 because 2D space
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	// set the angular velocity to 0 because only moving straight
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	// t0: current time
	// ros::Time::now().toSec gives the current time as an object
	// the .toSec() gives the current time as seconds
	double t0 = ros::Time::now().toSec();
	double current_distance = 0;
	// Rate object, 10 messages are sent per second
	ros::Rate loop_rate(100);

	// loop 
	// publish the velocity 
	// estimate the distance = speed * (t1 - t0)
	// current_distance_moved_by_robot <= distance
	// while the current distance is less than the argument distance
	// keep performing the loop
	do {
		// publish the velocity message
		velocity_publisher.publish(vel_msg);
		// measure the current distance
		double t1 = ros::Time::now().toSec();
		// estimate current distance moved by robot
		current_distance = speed * (t1-t0);
		// in order for ros to publish the message, must invoke ros::spinOnce()
		ros::spinOnce();
		loop_rate.sleep();

	}while(current_distance < distance);

	// force the robot to stop immediately 
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);


}

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
	// counter-clockwise = positive angle
	// clockwise = negative angle
	geometry_msgs::Twist vel_msg;

	double current_angle = 0.0;



	// set a random linear velocity in the x-axis
	// linear velocity components are all set to 0
	// because not performing any linear motion
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	// set a random angular velocity in the y-axis
	// perform 90 degree motion in angular z
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	// set the angular velocity
	if (clockwise) {
		vel_msg.angular.z = -abs(angular_speed);
	}
	else {
		vel_msg.angular.z = abs(angular_speed);
	}

	// the time before entering into the loop
	double t0 = ros::Time::now().toSec();

	// loop will execute 10 times per second
	ros::Rate loop_rate(10);

	// publish the velocity message
	// if we just publish the velocity message, the robot will
	// not stop rotating
	do {
		velocity_publisher.publish(vel_msg);

		// the current time
		double t1 = ros::Time::now().toSec();

		current_angle = angular_speed * (t1 - t0);

		ros::spinOnce();

		loop_rate.sleep();

	}while(current_angle < relative_angle);

	// force stop of the robot
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees) 
{
	return angle_in_degrees * (PI / 180.0);
}

double setDesiredOrientation(double desired_angle_radians)
{
	// to find the relative angle, substract the turtlesim_pose.theta 
	// (the current orientation of robot) from the desired_angle_radians
	// how did we get the turtlesim pose? we subscribed to the turtlesim_pose topic
	// 
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians < 0)? true:false);
	// cout << desired_angle_radians << "," << turtlesim_pose.theta << "," << relative_angle_radians;
	rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

void poseCallback(const turtlesim::Pose::ConstPtr &pose_message)
{
	// this is setting the turtlesim_pose to the be last updated message
	// from the callback function
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}