/*
 * sphero_controller.h
 *
 *  Created on: December 27, 2017
 *      Author: klchow
 */

#ifndef SPHEROCONTROLLER_H_
#define SPHEROCONTROLLER_H_

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "robot_code/RobotStatus.h"

#include "math.h"
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <cmath>
#include <Eigen/Dense>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

struct ControllerParams {
        double p_gain_linear_velocity;
        double i_gain_linear_velocity;
        double d_gain_linear_velocity;
        double p_gain_position;
        double i_gain_position;
        double d_gain_position;
        double p_gain_cross_track;
        double i_gain_cross_track;
        double d_gain_cross_track;
        double desired_max_velocity;
        double desired_max_accel;
        double max_waypoint_step_size;
        double look_ahead_distance;
        double goal_threshold;
        double vel_at_goal_threshold;
        double ang_vel_at_goal_threshold;
};

class SpheroController {
private:
	ros::Publisher controller_pub;
	ros::Publisher robot_status_pub;
	ros::Subscriber pose_sub;
	ros::Subscriber twist_sub;
	geometry_msgs::Twist twist_command;
        std::vector<std::vector<double> > input_waypoints;
        std::vector<std::vector<double> > waypoints;
        std::vector<std::vector<double> > velocity_waypoints;
        geometry_msgs::Pose2D current_pose;
        geometry_msgs::Twist current_twist;
        ControllerParams cp;
        double desired_velocity_x;
        double desired_velocity_y;
        double position_error_x;
        double position_error_y;
        double total_position_error_x;
        double total_position_error_y;
        double prev_position_error_x;
        double prev_position_error_y;
        double deriv_position_error_x;
        double deriv_position_error_y;
        double velocity_error_x;
        double velocity_error_y;
        double vel_x_command;
        double vel_y_command;
        double vel_x_ff;
        double vel_y_ff;
        double cross_track_error_x;
        double cross_track_error_y;
        int prev_min_distance_ind;
        int min_distance_ind;
        double prev_cross_track_error_x;
        double prev_cross_track_error_y;
        double deriv_cross_track_error_x;
        double deriv_cross_track_error_y;
        double total_cross_track_error_x;
        double total_cross_track_error_y;

public:
        // initialize controller
	SpheroController(ControllerParams controller_params, std::string node_name);

	~SpheroController();

        void SetWaypoints(const std::vector<std::vector<double> >& waypoint_array);
 
        std::vector<std::vector<double> > GetWaypoints();

        void UpdateCurrentPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

        void UpdateCurrentTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);

        geometry_msgs::Pose2D GetCurrentPose();

        void GenerateWaypointSpline();
        
        void GenerateVelocityWaypointSpline();

        void FindNearestWaypoint();

        void CalculateDesiredVelocity();
 
        void CalculateVelocityError();

        void CalculatePositionError();
  
        void CalculateFeedforwardCommands();

        void CalculateTwistCommand();
  
        geometry_msgs::Twist GetTwistCommand();
 
        void PublishTwistCommand();

        void PublishCurrentRobotStatus();

        bool ReachedGoal();

};

#endif /* SPHEROCONTROLLER_H_ */
