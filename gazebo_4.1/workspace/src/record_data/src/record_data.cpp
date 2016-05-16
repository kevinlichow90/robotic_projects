
#include "ros/ros.h"
#include "two_wheeled_robot_diff_drive_control/RobotStatus.h"
#include <fstream>
#include <iostream>
#include <string>

std::string filename = "two_wheeled_robot_test_051516";
std::ofstream outf(filename.c_str());

void UpdateCurrentRobotStatus(const two_wheeled_robot_diff_drive_control::RobotStatus::ConstPtr& msg) {
	// this function may change depending on what robot is called because different robots have different robot status messages
	// this will be for the two_wheeled_robot status message

	// first write the current time
	outf << ros::Time::now() << ", ";
	// next write the current pose2D
	outf << msg->pose_2d[0].x << ", ";
	outf << msg->pose_2d[0].y << ", ";
	outf << msg->pose_2d[0].theta << ", ";
	// then write the goal pose2D
	outf << msg->pose_2d[1].x << ", ";
    outf << msg->pose_2d[1].y << ", ";
    outf << msg->pose_2d[1].theta << ", ";
    // finally write the velocity command
    outf << msg->velocity_command[0].linear.x << ", ";
    outf << msg->velocity_command[0].angular.z << "\n";

}


int main(int argc, char* argv[]) {

	if (!ros::isInitialized()) {
	    std::string name = "record_data_node";
		int argc = 0;
		ros::init(argc, NULL, name, ros::init_options::NoSigintHandler);
	}

	ros::NodeHandle nh;

    ros::Subscriber robot_status_sub = nh.subscribe("/current_robot_status", 1000, UpdateCurrentRobotStatus);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
    	ros::spinOnce();
    	loop_rate.sleep();
    }
}
