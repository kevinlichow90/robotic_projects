/*
 * UnicycleModelControl.cpp
 *
 *  Created on: Mar 19, 2016
 *      Author: klchow
 */

#include "UnicycleModelControl.h"
#define PI 3.14159

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
/*
UnicycleModelControl::UnicycleModelControl() {
    if (!ros::isInitialized()){
	    std::string name = "unicycle_model_control_node";
	    int argc = 0;
	    ros::init(argc,NULL,name,ros::init_options::NoSigintHandler);
	}

    ros::NodeHandle m_nh;

    m_controller_pub = m_nh.advertise<geometry_msgs::Twist>("/gazebo/differential_drive_plugin_node/differential_drive_velocity_command", 1000);

    //m_controller_sub = m_nh.subscribe("/two_wheeled_robot/odom", 1000, &UnicycleModelControl::UpdateCurrentPoseCallback, this);
    m_controller_sub = m_nh.subscribe("/gazebo/differential_drive_plugin_node/differential_drive_pose", 1000, &UnicycleModelControl::UpdateCurrentPoseCallback, this);
    m_gain_angular_velocity = 0.65;
    //m_gain_angular_velocity = 0.1;
    m_gain_velocity = 0.1;
    //m_gain_velocity = 0.;
    //m_stop_orientation_control_distance = 0.0001;
    m_stop_orientation_control_distance = 0.;

    // should the error thresholds be a percentage of the distance between the goal and initial positions?
    m_error_r_threshold = 0.1;
    m_error_theta_threshold = 0.01;

}
*/

UnicycleModelControl::UnicycleModelControl(double gain_angular_velocity = NULL, double gain_velocity = NULL) : m_gain_angular_velocity(gain_angular_velocity), m_gain_velocity(gain_velocity) {
    if (!ros::isInitialized()){
	    std::string name = "unicycle_model_control_node";
	    int argc = 0;
	    ros::init(argc,NULL,name,ros::init_options::NoSigintHandler);
	}

    ros::NodeHandle m_nh;

    m_controller_pub = m_nh.advertise<geometry_msgs::Twist>("/gazebo/differential_drive_plugin_node/differential_drive_velocity_command", 1000);
    m_robot_status_pub = m_nh.advertise<two_wheeled_robot_diff_drive_control::RobotStatus>("/current_robot_status", 1000);

    //m_controller_sub = m_nh.subscribe("/two_wheeled_robot/odom", 1000, &UnicycleModelControl::UpdateCurrentPoseCallback, this);
    m_controller_sub = m_nh.subscribe("/gazebo/differential_drive_plugin_node/differential_drive_pose", 1000, &UnicycleModelControl::UpdateCurrentPoseCallback, this);
    //m_stop_orientation_control_distance = 0.0001;
    m_stop_orientation_control_distance = 0.;

    if (gain_angular_velocity == NULL) {
    	m_gain_angular_velocity = 0.65;
    }
    if (gain_velocity == NULL) {
    	m_gain_velocity = 0.1;
    }

    // should the error thresholds be a percentage of the distance between the goal and initial positions?
    m_error_r_threshold = 0.1;

}

UnicycleModelControl::~UnicycleModelControl() {
	// TODO Auto-generated destructor stub
}

void UnicycleModelControl::SetGoal(geometry_msgs::Pose2D goal_pose) {
	m_goal_pose = goal_pose;
}

geometry_msgs::Pose2D UnicycleModelControl::GetGoal() {
	return m_goal_pose;
}

bool UnicycleModelControl::ReachedGoal() {
	double error_r = pow(pow(m_goal_pose.x-m_current_pose.x,2)+pow(m_goal_pose.y-m_current_pose.y,2),0.5);
    if (error_r < m_error_r_threshold) {
    	return true;
    }
    else {
    	return false;
    }
}


void UnicycleModelControl::SendControllerCommand() {
	//make sure the current pose is locked somehow?
	geometry_msgs::Twist twist_command = UnicycleModelControl::CalculateWheelVelocities();
	m_twist_command = twist_command; // assign a member variable so that PublishCurrentRobotStatus can see the current twist command
	UnicycleModelControl::PublishWheelVelocities(twist_command);
}

// this should always be called after the member variables have already been set at least once
void UnicycleModelControl::PublishCurrentRobotStatus() {
	two_wheeled_robot_diff_drive_control::RobotStatus current_robot_status_msg;
	// defining local variables for current and goal pose for encapsulation and for my own clarity
	std::vector<geometry_msgs::Pose2D> poses;
	poses.push_back(m_current_pose);
    poses.push_back(m_goal_pose);
	std::vector<geometry_msgs::Twist> current_twist_command;
	current_twist_command.push_back(m_twist_command);
	current_robot_status_msg.pose_2d = poses;
	current_robot_status_msg.velocity_command = current_twist_command;

    m_robot_status_pub.publish(current_robot_status_msg);
}

/*
void UnicycleModelControl::UpdateCurrentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_current_pose.x = msg->pose.pose.position.x;
    m_current_pose.y = msg->pose.pose.position.y;
    //convert quaternion to z rotation
    double quaternion_x = msg->pose.pose.orientation.x;
    double quaternion_y = msg->pose.pose.orientation.y;
    double quaternion_z = msg->pose.pose.orientation.z;
    double quaternion_w = msg->pose.pose.orientation.w;

    //convert quaternion to angle axis and then to 2D angle
    double axis[3] = {quaternion_x/sqrt(1-quaternion_w/quaternion_w),
    		           quaternion_y/sqrt(1-quaternion_w/quaternion_w),
    		           quaternion_z/sqrt(1-quaternion_w/quaternion_w)};
    double angle = 2*acos(quaternion_w);

    //create m_current_pose.theta
}
*/

void UnicycleModelControl::UpdateCurrentPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    this->m_current_pose.x = msg->position.x;
    this->m_current_pose.y = msg->position.y;
    //convert quaternion to z rotation
    double quaternion_x = msg->orientation.x;
    double quaternion_y = msg->orientation.y;
    double quaternion_z = msg->orientation.z;
    double quaternion_w = msg->orientation.w;

    //convert quaternion to angle axis and then to 2D angle
    double axis[3] = {quaternion_x/sqrt(1-quaternion_w*quaternion_w),
    		           quaternion_y/sqrt(1-quaternion_w*quaternion_w),
    		           quaternion_z/sqrt(1-quaternion_w*quaternion_w)};
    double angle = 2*acos(quaternion_w)*sgn(axis[2]); // negative because that's how the signs turn out

    //create m_current_pose.theta
    this->m_current_pose.theta = angle;
    std::cout << "Got Pose message: " << m_current_pose.x << ", " << m_current_pose.y << ", " << m_current_pose.theta << "\n";
}

geometry_msgs::Twist UnicycleModelControl::CalculateWheelVelocities() {


	// try scaling the orientation control based on distance to goal (normalized by initial distance error)
	// doesn't work so well...try deadzone
	if (!m_initial_error_r) {
		//m_initial_error_r = pow(pow(m_goal_pose.x,2)+pow(m_goal_pose.y,2),0.5)-pow(pow(m_current_pose.x,2)+pow(m_current_pose.y,2),0.5);
		m_initial_error_r = pow(pow(m_goal_pose.x-m_current_pose.x,2)+pow(m_goal_pose.y-m_current_pose.y,2),0.5);
	    // does not maintain sign?
	}

	double goal_theta = atan2(m_goal_pose.y-m_current_pose.y, m_goal_pose.x-m_current_pose.x);
	double error_angular_position = goal_theta-m_current_pose.theta;
	//double goal_r = pow(pow(m_goal_pose.x,2)+pow(m_goal_pose.y,2),0.5);
	//double current_r = pow(pow(m_current_pose.x,2)+pow(m_current_pose.y,2),0.5);
	//double error_r = goal_r-current_r;
	double error_r = pow(pow(m_goal_pose.x-m_current_pose.x,2)+pow(m_goal_pose.y-m_current_pose.y,2),0.5);
	// does not maintain sign? could just run in reverse instead of turnning around
	//double desired_angular_velocity = 0;
	//if (error_r > m_stop_orientation_control_distance) {
	//	double desired_angular_velocity = m_gain_angular_velocity*error_angular_position;
	//}

	double desired_angular_velocity = m_gain_angular_velocity*error_angular_position;
	double desired_velocity = m_gain_velocity*error_r;

	// if it is easier to reverse to the desired position, then reverse
	// only makes sense if it does not matter which direction the robot faces
	if ((error_angular_position > (PI/2)) || (error_angular_position < -(PI/2))) {
		if (error_angular_position > (PI/2)) {
		    desired_angular_velocity = m_gain_angular_velocity*(error_angular_position-PI);
		}
		else if (error_angular_position < -(PI/2)) {
			desired_angular_velocity = m_gain_angular_velocity*(PI+error_angular_position);
		}
	    desired_velocity *= -1;
	}

	std::cout << "m_goal_pose.x: " << m_goal_pose.x << "\n";
	std::cout << "m_goal_pose.y: " << m_goal_pose.y << "\n";
	std::cout << "m_goal_pose.theta: " << goal_theta << "\n";
	//std::cout << "goal_r: " << goal_r << "\n";
	std::cout << "m_current_pose.x: " << m_current_pose.x << "\n";
	std::cout << "m_current_pose.y: " << m_current_pose.y << "\n";
	std::cout << "m_current_pose.theta: " << m_current_pose.theta << "\n";
	//std::cout << "current_r: " << current_r << "\n";
	std::cout << "error_r: " << error_r << "\n";
	std::cout << "desired_angular_velocity: " << desired_angular_velocity << "\n";
	std::cout << "desired_velocity: " << desired_velocity << "\n";

	// does the differential drive controller take the absolute value of the desired velocity?
	geometry_msgs::Twist twist_command;
	twist_command.linear.x = desired_velocity;
	twist_command.angular.z = desired_angular_velocity;
	return twist_command;
}

void UnicycleModelControl::PublishWheelVelocities(geometry_msgs::Twist twist_command) {
	m_controller_pub.publish(twist_command);
}




