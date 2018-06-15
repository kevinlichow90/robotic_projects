/*
 * two_wheeld_robot_controller.h
 *
 *  Created on: December 27, 2017
 *      Author: klchow
 */

#ifndef TWOWHEELEDROBOTCONTROLLER_H_
#define TWOWHEELEDROBOTCONTROLLER_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "robot_code/RobotStatus.h"

#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <Eigen/Dense>

#define PI 3.14159265358979323846  /* pi */

struct ControllerParams {
        double p_gain_position;
        double i_gain_position;
        double d_gain_position;
        double p_gain_orientation;
        double p_gain_cross_track;
        double i_gain_cross_track;
        double d_gain_cross_track;
        double desired_max_velocity;
        double max_waypoint_step_size;
        double look_ahead_distance;
        double goal_threshold;
};

class TwoWheeledRobotController {
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
        double desired_velocity;
        double velocity_error;
        double velocity_command;
        double velocity_ff;
        double desired_angle;
        double prev_cross_track_error;
        double cross_track_error;
        double deriv_cross_track_error;
        double total_cross_track_error;
        double angle_ff;
        double angular_velocity_command;
        double prev_position_error;
        double position_error_x;
        double position_error_y;
        double position_error;
        double total_position_error;
        double deriv_position_error;
        double angular_error;
        double waypoint_angle_error;
        int prev_min_distance_ind;
        int min_distance_ind;

public:
	TwoWheeledRobotController(ControllerParams controller_param);

	 ~TwoWheeledRobotController();

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
 
        void UpdateControllerCalculations();

        bool ReachedGoal();

};

#endif /* TWOWHEELEDROBOTCONTROLLER_H_ */
