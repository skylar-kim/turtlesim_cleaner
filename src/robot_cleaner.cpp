#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// create the publisher as a global variable
ros::Publisher velocity_publisher;

// method to move the robot straight forward
void move(double speed, double distance, bool isForward);

using namespace std;

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;

	double speed;
	double distance;
	bool isForward;
	// declare the publisher 
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

	cout << "Enter your speed: " << endl;
	cin >> speed;
	cout << "Enter your distance: " << endl;
	cin >> distance;
	cout << "Forward?: " << endl;
	cin >> isForward;

	move(speed, distance, isForward);
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