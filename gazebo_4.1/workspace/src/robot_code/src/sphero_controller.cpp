/*
 * sphero_controller.cpp
 *
 *  Created on: December 27, 2017
 *      Author: klchow
 */

#include "sphero_controller.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

std::vector<std::string> split_string_delim(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

double calculate_total_path_length(std::vector<std::vector<double> >& v) {
// find the diff of each index of the vectors and find the norm of the difference
    double path_length = 0;
    std::vector<double> diff_vector_single;
    for (int iii = 0; iii < (v.size()-1); iii++) {
         path_length += sqrt(pow(v[iii+1][0]-v[iii][0],2)+pow(v[iii+1][1]-v[iii][1],2));
    }
    return path_length;
} 


SpheroController::SpheroController(ControllerParams controller_params, std::string node_name) : cp(controller_params) {
    if (!ros::isInitialized()){
            int argc = 0;
            ros::init(argc,NULL,node_name,ros::init_options::NoSigintHandler);
        }

    ros::NodeHandle nh;

    total_position_error_x = 0;
    total_position_error_y = 0;
    cross_track_error_x = 0;
    cross_track_error_y = 0;
    total_cross_track_error_y = 0;
    total_cross_track_error_y = 0;
    nearest_waypoint_ind = 0;
    prev_nearest_waypoint_ind = 0;

    std::vector<std::string> node_name_split_vec = split_string_delim(node_name,"_");

    controller_pub = nh.advertise<geometry_msgs::Twist>("/gazebo/"+node_name_split_vec[0]+"_drive_plugin_node/"+node_name_split_vec[0]+"_drive_velocity_command", 1000);
    robot_status_pub = nh.advertise<robot_code::RobotStatus>("/current_robot_status", 1000);

    pose_sub = nh.subscribe("/gazebo/"+node_name_split_vec[0]+"_drive_plugin_node/"+node_name_split_vec[0]+"_drive_pose", 1000, &SpheroController::UpdateCurrentPoseCallback, this);
    twist_sub = nh.subscribe("/gazebo/"+node_name_split_vec[0]+"_drive_plugin_node/"+node_name_split_vec[0]+"_drive_twist", 1000, &SpheroController::UpdateCurrentTwistCallback, this);
}

SpheroController::~SpheroController() {
}

void SpheroController::SetWaypoints(const std::vector<std::vector<double> >& waypoint_array){
    // put waypoints into members
    input_waypoints = waypoint_array;
}

std::vector<std::vector<double> > SpheroController::GetWaypoints(){
    return waypoints;
}

void SpheroController::UpdateCurrentPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
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
    //std::cout << "Got Pose message: " << current_pose.x << ", " << current_pose.y << ", " << current_pose.theta << "\n";

}


void SpheroController::UpdateCurrentTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    current_twist.linear.x = msg->linear.x;
    current_twist.linear.y = msg->linear.y;
    current_twist.linear.z = msg->linear.z;
    current_twist.angular.x = msg->angular.x;
    current_twist.angular.y = msg->angular.y;
    current_twist.angular.z = msg->angular.z;
}

geometry_msgs::Pose2D SpheroController::GetCurrentPose() {
    return current_pose;
}


void SpheroController::GenerateWaypointSpline() {
    // calculate spline based on waypoints
    // try straight line btwn initial position and goal first, then use waypoints
    if (waypoints.size() == 2) {
        // first try chopping up path into discrete waypoints
        int num_of_waypoints = 100; // must be greater than 2
        double step_size_x = (waypoints[1][0]-waypoints[0][0])/(num_of_waypoints-1);
        double step_size_y = (waypoints[1][1]-waypoints[0][1])/(num_of_waypoints-1);
        double waypoint_x = waypoints[0][0];
        double waypoint_y = waypoints[0][1];
        for (int iii = 0; iii < num_of_waypoints-2; iii++) {
            waypoint_x += step_size_x;
            waypoint_y += step_size_y;
            std::vector<double> waypoint_x_y;
            waypoint_x_y.push_back(waypoint_x);
            waypoint_x_y.push_back(waypoint_y);
            waypoints.insert(waypoints.end()-1,waypoint_x_y);
        }
    } else{
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
                // do something
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
}

void SpheroController::GenerateVelocityWaypointSpline() {
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

void SpheroController::FindNearestWaypoint() {
    // find nearest waypoint to current position
    int min_distance_ind = 0;
    double current_norm_distance = sqrt(pow(current_pose.x,2)+pow(current_pose.y,2));
    double min_distance = current_norm_distance+calculate_total_path_length(waypoints); // change to be slightly larger than full range 
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
    if (prev_nearest_waypoint_ind == (waypoints.size()-1)) {
        end_ind = waypoints.size()-1;
    }
    else if (prev_nearest_waypoint_ind+look_ahead_inds > (waypoints.size()-1)) {
        end_ind = waypoints.size()-1;
    } else {
        end_ind = prev_nearest_waypoint_ind+look_ahead_inds;
    }
    for (int iii = prev_nearest_waypoint_ind; iii <= end_ind; iii++) {
        double distance = sqrt(pow(current_pose.x-waypoints[iii][0],2)+pow(current_pose.y-waypoints[iii][1],2));
        if (distance < min_distance) {
            min_distance_ind = iii;
            min_distance = distance;
        }
    }
    this->prev_nearest_waypoint_ind = this->nearest_waypoint_ind;
    this->nearest_waypoint_ind = min_distance_ind;
}


void SpheroController::CalculatePositionError() {
    // subtract desired velocity from current velocity
    std::vector<double> nearest_waypoint = waypoints[nearest_waypoint_ind];
    prev_position_error_x = position_error_x;
    prev_position_error_y = position_error_y;
    position_error_x = nearest_waypoint[0]-current_pose.x; 
    position_error_y = nearest_waypoint[1]-current_pose.y; 
    total_position_error_x += position_error_x;
    total_position_error_y += position_error_y;
    deriv_position_error_x = position_error_x-prev_position_error_x;
    deriv_position_error_y = position_error_y-prev_position_error_y;

    // calculate cross track error
    // determine orthogonal unit vector btwn current position and velocity vector
    // dot proudct btwn orthogonal unit vector and current position error to find cross track error
    std::vector<double> unit_vel;
    unit_vel.push_back(velocity_waypoints[nearest_waypoint_ind][0]);
    unit_vel.push_back(velocity_waypoints[nearest_waypoint_ind][1]);
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

    Eigen::Vector3d position_error(position_error_x, position_error_y, 0);
    Eigen::Vector3d cross_track_error = position_error.cross(a1_vec); //****check sign 

    prev_cross_track_error_x = cross_track_error_x;
    prev_cross_track_error_y = cross_track_error_y;
    cross_track_error_x = a2_vec[0]*cross_track_error[2]; //****norm?
    cross_track_error_y = a2_vec[1]*cross_track_error[2];

    deriv_cross_track_error_x = cross_track_error_x-prev_cross_track_error_x;
    deriv_cross_track_error_y = cross_track_error_y-prev_cross_track_error_y;

    total_cross_track_error_x += cross_track_error_x;
    total_cross_track_error_y += cross_track_error_y;

    std::cout << "cross track error: " << cross_track_error_x << "\t" << cross_track_error_y << "\n";
    std::cout << "deriv cross track error: " << deriv_cross_track_error_x << "\t" << deriv_cross_track_error_y << "\n";
    std::cout << "total cross track error: " << total_cross_track_error_x << "\t" << total_cross_track_error_y << "\n";
    std::cout << "position error: " << position_error_x << "\t" << position_error_y << "\n";
    std::cout << "deriv position error: " << deriv_position_error_x << "\t" << deriv_position_error_y << "\n";
    std::cout << "total position error: " << total_position_error_x << "\t" << total_position_error_y << "\n";
}

void SpheroController::CalculateFeedforwardCommands() {
    // should eventually have a velocity waypoint vector also 
    /*
    if (nearest_waypoint == waypoints[0]) {
        vel_x_ff = cp.desired_max_velocity/2;
        vel_y_ff = cp.desired_max_velocity/2;
    } else if (nearest_waypoint == waypoints[-1]) {
        vel_x_ff = 0;
        vel_y_ff = 0;
    } else {
        vel_x_ff = cp.desired_max_velocity;
        vel_y_ff = cp.desired_max_velocity;
    }
    */
    desired_velocity_x = velocity_waypoints[nearest_waypoint_ind][0]; 
    desired_velocity_y = velocity_waypoints[nearest_waypoint_ind][1]; 
}

void SpheroController::CalculateTwistCommand() {
    // function that calculates twist command to robot - composed of angular and linear velocity commands
        // angular velocity command
            // feedforward orientation based on path 
            // PID control law for cross track error
        // linear velocity command
            // feedforward velocity based on trajectory
            // PID control law for velocity error

    std::cout << "nearest waypoint: " << waypoints[nearest_waypoint_ind][0] << "\t" << waypoints[nearest_waypoint_ind][1] << "\n";

    if (nearest_waypoint_ind == (waypoints.size()-1)) {
        velocity_command_x = cp.p_gain_position*(position_error_x)+cp.d_gain_position*(deriv_position_error_x);//+cp.i_gain_position*(total_position_error_x);
        velocity_command_y = cp.p_gain_position*(position_error_y)+cp.d_gain_position*(deriv_position_error_y);//+cp.i_gain_position*(total_position_error_y);
    } else {
        velocity_command_x = desired_velocity_x+cp.p_gain_cross_track*(cross_track_error_x)+cp.d_gain_cross_track*(deriv_cross_track_error_x)+cp.i_gain_cross_track*(total_cross_track_error_x);
        velocity_command_y = desired_velocity_y+cp.p_gain_cross_track*(cross_track_error_y)+cp.d_gain_cross_track*(deriv_cross_track_error_y)+cp.i_gain_cross_track*(total_cross_track_error_y);
        std::cout << cp.p_gain_cross_track << "\t" << cp.d_gain_cross_track << "\n";
        std::cout << cp.i_gain_cross_track << "\t" << total_cross_track_error_x << "\t" << total_cross_track_error_y << "\n";
        std::cout << "total cross track command: " << cp.i_gain_cross_track*(total_cross_track_error_x) << "\t" << cp.i_gain_cross_track*(total_cross_track_error_y) << "\n";
    }
    twist_command.linear.x = velocity_command_x;
    twist_command.linear.y = velocity_command_y;
    
    // time based trajectory following vs position based
    // position based: chrome-extension://oemmndcbldboiebfnladdacbdfmadadm/https://www.ri.cmu.edu/pub_files/pub4/singh_sanjiv_1989_1/singh_sanjiv_1989_1.pdf

}

geometry_msgs::Twist SpheroController::GetTwistCommand() {
    return twist_command;
}

void SpheroController::PublishTwistCommand() {
    controller_pub.publish(twist_command);
}

void SpheroController::PublishCurrentRobotStatus() {
    robot_code::RobotStatus current_robot_status_msg;
    
    std::vector<geometry_msgs::Pose2D> poses;
    poses.push_back(current_pose);
    std::vector<geometry_msgs::Twist> current_twist_command;
    current_twist_command.push_back(twist_command);
    current_robot_status_msg.pose_2d = poses;
    current_robot_status_msg.velocity_command = current_twist_command;

    robot_status_pub.publish(current_robot_status_msg);

}

void SpheroController::UpdateControllerCalculations() {
    FindNearestWaypoint();
    CalculatePositionError();
    CalculateFeedforwardCommands();
    CalculateTwistCommand();
    ReachedGoal();
    PublishTwistCommand();
}

bool SpheroController::ReachedGoal() {
    if (nearest_waypoint_ind == (waypoints.size()-1)) {
        double distance_to_goal = sqrt(pow(current_pose.x-waypoints[waypoints.size()-1][0],2)+pow(current_pose.y-waypoints[waypoints.size()-1][1],2));
        std::cout << "distance_to_goal: " << distance_to_goal << "\n";
        if (distance_to_goal<cp.goal_threshold) {
           // publish velocity to zero
           twist_command.linear.x = 0;
           twist_command.linear.y = 0;
           std::cout << "******************************COMPLETED****************************" << "\n";
           return 1;
        }
    }
    return 0;
}
