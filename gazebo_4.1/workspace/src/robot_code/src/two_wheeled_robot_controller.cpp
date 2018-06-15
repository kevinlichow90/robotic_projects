/*
 * two_wheeled_robot_controller.cpp
 *
 *  Created on: December 27, 2017
 *      Author: klchow
 */

#include "two_wheeled_robot_controller.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

inline double wrapAngle( double angle )
{
    double twoPi = 2.0 * PI;
    return angle - twoPi * floor( angle / twoPi );
}

TwoWheeledRobotController::TwoWheeledRobotController(ControllerParams controller_params) : cp(controller_params) {
    if (!ros::isInitialized()){
            int argc = 0;
            ros::init(argc,NULL,"two_wheeled_robot_controller",ros::init_options::NoSigintHandler);
        }

    ros::NodeHandle nh;

    cross_track_error = 0;
    min_distance_ind = 0;
    prev_min_distance_ind = 0;
    position_error = 0;
    prev_position_error = 0;

    controller_pub = nh.advertise<geometry_msgs::Twist>("/gazebo/differential_drive_plugin_node/differential_drive_velocity_command", 1000);
    robot_status_pub = nh.advertise<robot_code::RobotStatus>("/current_robot_status", 1000);

    pose_sub = nh.subscribe("/gazebo/differential_drive_plugin_node/differential_drive_pose", 1000, &TwoWheeledRobotController::UpdateCurrentPoseCallback, this);
    twist_sub = nh.subscribe("/gazebo/differential_drive_plugin_node/differential_drive_twist", 1000, &TwoWheeledRobotController::UpdateCurrentTwistCallback, this);
}

TwoWheeledRobotController::~TwoWheeledRobotController() {
}

void TwoWheeledRobotController::SetWaypoints(const std::vector<std::vector<double> >& waypoint_array){
    // put waypoints into members
    input_waypoints = waypoint_array;
}

std::vector<std::vector<double> > TwoWheeledRobotController::GetWaypoints(){
    return waypoints;
}

void TwoWheeledRobotController::UpdateCurrentPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    current_pose.x = msg->position.x;
    current_pose.y = msg->position.y;
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

    current_pose.theta = angle;
    //std::cout << "Got Pose message: " << m_current_pose.x << ", " << m_current_pose.y << ", " << m_current_pose.theta << "\n";

}

void TwoWheeledRobotController::UpdateCurrentTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    current_twist.linear.x = msg->linear.x;
    current_twist.linear.y = msg->linear.y;
    current_twist.linear.z = msg->linear.z;
    current_twist.angular.x = msg->angular.x;
    current_twist.angular.y = msg->angular.y;
    current_twist.angular.z = msg->angular.z;
}

geometry_msgs::Pose2D TwoWheeledRobotController::GetCurrentPose() {
    return current_pose;
}

void TwoWheeledRobotController::GenerateWaypointSpline() {
    // calculate spline based on waypoints
    // try straight line btwn initial position and goal first, then use waypoints
    waypoints.push_back(input_waypoints[0]);
    for (int iii = 0; iii < (input_waypoints.size()-1); iii++) {
        bool within_waypoint_threshold = 0;

        double waypoint_x = input_waypoints[iii][0];
        double waypoint_y = input_waypoints[iii][1];

        std::vector<double> unit_vec;
        unit_vec.push_back(input_waypoints[iii+1][0]-input_waypoints[iii][0]);
        unit_vec.push_back(input_waypoints[iii+1][1]-input_waypoints[iii][1]);
        double vec_norm = sqrt(pow(unit_vec[0],2)+pow(unit_vec[1],2));
        unit_vec[0] = unit_vec[0]/vec_norm;
        unit_vec[1] = unit_vec[1]/vec_norm;

        std::cout << input_waypoints[iii+1][0] << "\t" << input_waypoints[iii][0] << "\t" << input_waypoints[iii+1][1] << "\t" << input_waypoints[iii][1] << "\n";
        std::cout << unit_vec[0] << "   " << unit_vec[1] << "\n";

        while (!within_waypoint_threshold) {
            waypoint_x += cp.max_waypoint_step_size*unit_vec[0];
            waypoint_y += cp.max_waypoint_step_size*unit_vec[1];
            std::vector<double> waypoint_x_y;
            waypoint_x_y.push_back(waypoint_x);
            waypoint_x_y.push_back(waypoint_y);
            waypoints.push_back(waypoint_x_y);

            within_waypoint_threshold = sqrt(pow(waypoint_x-input_waypoints[iii+1][0],2)+pow(waypoint_y-input_waypoints[iii+1][1],2))<cp.max_waypoint_step_size;

        }
        waypoints.push_back(input_waypoints[iii+1]);
    }
}

void TwoWheeledRobotController::GenerateVelocityWaypointSpline() {
    // make sure position waypoint spline has already been generated
    for (int iii = 0; iii < waypoints.size()-1; iii++) {
        std::vector<double> unit_vel;
        unit_vel.push_back(waypoints[iii+1][0]-waypoints[iii][0]);
        unit_vel.push_back(waypoints[iii+1][1]-waypoints[iii][1]);
        double norm_vel = sqrt(pow(unit_vel[0],2)+pow(unit_vel[1],2));
        unit_vel[0] = unit_vel[0]/norm_vel;
        unit_vel[1] = unit_vel[1]/norm_vel;

        double desired_vel_x, desired_vel_y;
        if (iii == 0 || iii == waypoints.size()-1 || iii == waypoints.size()-2) {
           desired_vel_x = unit_vel[0]*cp.desired_max_velocity/2;
           desired_vel_y = unit_vel[1]*cp.desired_max_velocity/2;
        } else {
           desired_vel_x = unit_vel[0]*cp.desired_max_velocity;
           desired_vel_y = unit_vel[1]*cp.desired_max_velocity;
        }
        std::vector<double> desired_vel;
        desired_vel.push_back(desired_vel_x);
        desired_vel.push_back(desired_vel_y);
        velocity_waypoints.push_back(desired_vel);
    }
    std::vector<double> desired_vel;
    desired_vel.push_back(0);
    desired_vel.push_back(0);
    velocity_waypoints.push_back(desired_vel);
}


void TwoWheeledRobotController::FindNearestWaypoint() {
    // find nearest waypoint to current position
    int min_distance_ind = 0;
    double min_distance = 1000; // change to be slightly larger than full range 
    int look_ahead_inds = int (cp.look_ahead_distance/cp.max_waypoint_step_size);
    int start_ind, end_ind;
    /*
    if (prev_min_distance_ind == (waypoints.size()-1)) {
        start_ind = waypoints.size()-1;
    } else if(prev_min_distance_ind < look_ahead_inds) {
        start_ind = 0;
    } else {
        start_ind = prev_min_distance_ind-look_ahead_inds;
    }
    */
    if (prev_min_distance_ind == (waypoints.size()-1)) {
        end_ind = waypoints.size()-1;
    }
    else if (prev_min_distance_ind+look_ahead_inds > (waypoints.size()-1)) {
        end_ind = waypoints.size()-1;
    } else {
        end_ind = prev_min_distance_ind+look_ahead_inds;
    }
    for (int iii = prev_min_distance_ind; iii <= end_ind; iii++) {
        double distance = sqrt(pow(current_pose.x-waypoints[iii][0],2)+pow(current_pose.y-waypoints[iii][1],2));
        if (distance < min_distance) {
            min_distance_ind = iii;
            min_distance = distance;
        }
    }
    this->prev_min_distance_ind = this->min_distance_ind;
    this->min_distance_ind = min_distance_ind;
}


void TwoWheeledRobotController::CalculatePositionError() {
    // subtract desired velocity from current velocity
    std::vector<double> nearest_waypoint = waypoints[min_distance_ind];
    prev_position_error = position_error;
    position_error_x = nearest_waypoint[0]-current_pose.x;
    position_error_y = nearest_waypoint[1]-current_pose.y;
    position_error = sqrt(pow(nearest_waypoint[0]-current_pose.x,2)+pow(nearest_waypoint[1]-current_pose.y,2));
    total_position_error += position_error;
    deriv_position_error = position_error-prev_position_error;

    // calculate cross track error
    // determine orthogonal unit vector btwn current position and velocity vector
    // dot proudct btwn orthogonal unit vector and current position error to find cross track error
    std::vector<double> unit_vel;
    unit_vel.push_back(velocity_waypoints[min_distance_ind][0]);
    unit_vel.push_back(velocity_waypoints[min_distance_ind][1]);
    double norm_vel = sqrt(pow(unit_vel[0],2)+pow(unit_vel[1],2));
    //***** if statement to check if norm_vel is 0
    if (norm_vel==0) {
        unit_vel[0] = 0;
        unit_vel[1] = 0;
    } else {
        unit_vel[0] = unit_vel[0]/norm_vel;
        unit_vel[1] = unit_vel[1]/norm_vel;
    }
    Eigen::Vector3d a3_vec(0,0,1);
    Eigen::Vector3d a1_vec(unit_vel[0], unit_vel[1],0);
    Eigen::Vector3d a2_vec = a1_vec.cross(a3_vec);

    Eigen::Vector3d position_error_eigen(nearest_waypoint[0]-current_pose.x, nearest_waypoint[1]-current_pose.y, 0);
    Eigen::Vector3d cross_track_error_eigen = a1_vec.cross(position_error_eigen); //****check sign 

    prev_cross_track_error = cross_track_error;
    cross_track_error = cross_track_error_eigen[2]; //****norm?

    deriv_cross_track_error = cross_track_error-prev_cross_track_error;

    total_cross_track_error += cross_track_error;

    std::cout << "cross track error: " << cross_track_error  << "\n";
    std::cout << "deriv cross track error: " << deriv_cross_track_error << "\n";
    std::cout << "total cross track error: " << total_cross_track_error << "\n";
    std::cout << "position error: " << position_error << "\n";
    std::cout << "deriv position error: " << deriv_position_error << "\n";
    std::cout << "total position error: " << total_position_error << "\n";
}

void TwoWheeledRobotController::CalculateFeedforwardCommands() {
    // use velocity
    velocity_ff = sqrt(pow(velocity_waypoints[min_distance_ind][0],2)+pow(velocity_waypoints[min_distance_ind][1],2));
    angle_ff = atan2(velocity_waypoints[min_distance_ind][1],velocity_waypoints[min_distance_ind][0]);    // get angle_ff into range of 0 to 2*pi
    angle_ff = wrapAngle(angle_ff);
}

void TwoWheeledRobotController::CalculateTwistCommand() {
    // function that calculates twist command to robot - composed of angular and linear velocity commands
        // angular velocity command
            // feedforward orientation based on path 
            // PID control law for cross track error
        // linear velocity command
            // feedforward velocity based on trajectory
            // PID control law for velocity error
    std::cout << "nearest waypoint: " << waypoints[min_distance_ind][0] << "\t" << waypoints[min_distance_ind][1] << "\n";
    std::cout << "nearest velocity waypoint: " << velocity_waypoints[min_distance_ind][0] << "\t" << velocity_waypoints[min_distance_ind][1] << "\n";

    if (min_distance_ind == (waypoints.size()-1)) {
        // negative sign because I want to be facing toward the goal position
        angular_error = wrapAngle(atan2(position_error_y,position_error_x))-wrapAngle(current_pose.theta);
        std::cout << "position error: " << position_error_x << "\t" << position_error_y << "\n";
        std::cout << "angular error pre: " << angular_error << "\n";
        // can go forwards or backwards to reach goal
        double position_error_forw_back;
        double deriv_position_error_forw_back;
        double total_position_error_forw_back;
        double angular_error2;
        
        if (angular_error < 0) {
           angular_error2 = angular_error+2*PI;
           if (abs(angular_error) > abs(angular_error2)) {
               angular_error = angular_error2;
           }
        } else {
           angular_error2 = angular_error-2*PI;
           if (abs(angular_error) > abs(angular_error2)) {
               angular_error = angular_error2;
           }
        }
           
        position_error_forw_back = position_error;
        deriv_position_error_forw_back = deriv_position_error;
        total_position_error_forw_back = total_position_error;
        
        std::cout << "angular error post: " << angular_error << "\n";
        std::cout << "position error forward/backward: " << position_error_forw_back << "\n";
        std::cout << "deriv position error forward/backward: " << deriv_position_error_forw_back << "\n";
        std::cout << "total position error forward/backward: " << total_position_error_forw_back << "\n";
        velocity_command = cp.p_gain_position*(position_error_forw_back)+cp.d_gain_position*(deriv_position_error_forw_back);
        //velocity_command = 0;
        angular_velocity_command = cp.p_gain_orientation*angular_error;
    } else {
        velocity_command = velocity_ff;
        //velocity_command = 0;
        //****determine angular error
        angular_error = angle_ff-wrapAngle(current_pose.theta);
        double angular_error2;
        std::cout << "current_pose.theta: " << current_pose.theta << "\n";
        std::cout << "angle_ff: " << angle_ff << "\n";
        std::cout << "angular error pre: " << angular_error << "\n";
        if (angular_error < 0) {
           angular_error2 = angular_error+2*PI;
           if (abs(angular_error) > abs(angular_error2)) {
               angular_error = angular_error2;
           }
        } else {
           angular_error2 = angular_error-2*PI;
           if (abs(angular_error) > abs(angular_error2)) {
               angular_error = angular_error2;
           }
        }
        waypoint_angle_error = wrapAngle(atan2(position_error_y,position_error_x))-wrapAngle(current_pose.theta);
        double waypoint_angle_error2;
        if (waypoint_angle_error < 0) {
           waypoint_angle_error2 = waypoint_angle_error+2*PI;
           if (abs(waypoint_angle_error) > abs(waypoint_angle_error2)) {
               waypoint_angle_error = waypoint_angle_error2;
           }
        } else {
           waypoint_angle_error2 = waypoint_angle_error-2*PI;
           if (abs(waypoint_angle_error) > abs(waypoint_angle_error2)) {
               waypoint_angle_error = waypoint_angle_error2;
           }
        }
        // multiplying the angular_error term by absolute value of cross_track_error so that if cross track error is very large, this term will limit the cross track error term from getting to large
        //angular_velocity_command = cp.p_gain_orientation*(angular_error)*abs(cross_track_error)+cp.p_gain_cross_track*(cross_track_error)+cp.d_gain_cross_track*(deriv_cross_track_error);
        // this didn't work well
        // need to limit the cross track error term in case it gets very large somehow
        angular_velocity_command = cp.p_gain_orientation*(angular_error)+cp.p_gain_cross_track*(cross_track_error)+cp.d_gain_cross_track*(deriv_cross_track_error)+cp.p_gain_cross_track*(waypoint_angle_error);
        // this causes the robot to bounce around a lot, but still seems to work
        std::cout << "angular error post: " << angular_error << "\n";

    }
    twist_command.linear.x = velocity_command;
    twist_command.angular.z = angular_velocity_command;
    std::cout << "current pose: " << current_pose.x << "\t" << current_pose.y << "\t" << current_pose.theta << "\n";
}

geometry_msgs::Twist TwoWheeledRobotController::GetTwistCommand() {
    return twist_command;
}

void TwoWheeledRobotController::PublishTwistCommand() {
    controller_pub.publish(twist_command);
}

void TwoWheeledRobotController::PublishCurrentRobotStatus() {
    robot_code::RobotStatus current_robot_status_msg;
    
    std::vector<geometry_msgs::Pose2D> poses;
    poses.push_back(current_pose);
    std::vector<geometry_msgs::Twist> current_twist_command;
    current_twist_command.push_back(twist_command);
    current_robot_status_msg.pose_2d = poses;
    current_robot_status_msg.velocity_command = current_twist_command;

    robot_status_pub.publish(current_robot_status_msg);

}

void TwoWheeledRobotController::UpdateControllerCalculations() {
    FindNearestWaypoint();
    CalculatePositionError();
    CalculateFeedforwardCommands();
    CalculateTwistCommand();
    ReachedGoal();
    PublishTwistCommand();
}

bool TwoWheeledRobotController::ReachedGoal() {
    if (min_distance_ind == (waypoints.size()-1)) {
        double distance_to_goal = sqrt(pow(current_pose.x-waypoints[waypoints.size()-1][0],2)+pow(current_pose.y-waypoints[waypoints.size()-1][1],2));
        std::cout << "distance_to_goal: " << distance_to_goal << "\n";
        if (distance_to_goal<cp.goal_threshold) {
           // publish velocity to zero
           twist_command.linear.x = 0;
           twist_command.angular.z = 0;
           std::cout << "******************************COMPLETED****************************" << "\n";
           return 1;
        }
    }
    return 0;
}
