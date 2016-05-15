/*
 * UnicycleModelControl.h
 *
 *  Created on: Mar 19, 2016
 *      Author: klchow
 */

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "two_wheeled_robot_diff_drive_control/RobotStatus.h"

#ifndef UNICYCLEMODELCONTROL_H_
#define UNICYCLEMODELCONTROL_H_

#include "Controller.h"

class UnicycleModelControl {
private:

	geometry_msgs::Pose2D m_goal_pose;
	ros::Publisher m_controller_pub;
	ros::Publisher m_robot_status_pub;
	ros::Subscriber m_controller_sub;
	geometry_msgs::Pose2D m_current_pose;
	geometry_msgs::Twist m_twist_command;
	double m_gain_angular_velocity;
	double m_gain_velocity;
	double m_initial_error_r;
	double m_stop_orientation_control_distance;
	double m_error_r_threshold;

public:
	//UnicycleModelControl();
	UnicycleModelControl(double gain_angular_velocity, double gain_velocity);
	 ~UnicycleModelControl();

	void SetGoal(geometry_msgs::Pose2D goal_pose);
	geometry_msgs::Pose2D GetGoal();
    bool ReachedGoal();

	//void UpdateCurrentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void UpdateCurrentPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

	void SendControllerCommand();

	geometry_msgs::Twist CalculateWheelVelocities();

	void PublishWheelVelocities(geometry_msgs::Twist twist_command);

	void PublishCurrentRobotStatus();


};

#endif /* UNICYCLEMODELCONTROL_H_ */
