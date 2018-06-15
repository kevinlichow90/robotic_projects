
#include "ros/ros.h"
#include "two_wheeled_robot_controller.h"
#include <fstream>

// split a string at any spaces, commas, endlines, EOF
void split_string(std::string stringInput, std::vector<std::string>& vector_of_strings) {
    std::string temp_string = "";
    for (int i = 0; i < stringInput.length(); i++) {
        if ((stringInput[i] != ' ') && (stringInput[i] != '\n') && (stringInput[i] != ',') && (stringInput[i] != '\0')) {
            temp_string.push_back(stringInput[i]);
        }
        else {
            vector_of_strings.push_back(temp_string);
            if (stringInput[i] == '\0') {
                std::cout << "exiting" << "\n";
                return;
            }
            else {
                temp_string = "";
            }
        }
    }
    vector_of_strings.push_back(temp_string);
}


int main(int argc, char **argv) {

    ControllerParams controller_params;

    controller_params.p_gain_position = 1.;
    controller_params.i_gain_position = 0.01;
    controller_params.d_gain_position = 0.1;
    controller_params.p_gain_orientation = 1;
    controller_params.p_gain_cross_track = 0.1;
    controller_params.i_gain_cross_track = 0.01;
    controller_params.d_gain_cross_track = 10.;
    controller_params.desired_max_velocity = 0.3;
    controller_params.max_waypoint_step_size = 0.1;
    controller_params.look_ahead_distance = 0.5;
    controller_params.goal_threshold = 0.001;

    TwoWheeledRobotController c = TwoWheeledRobotController(controller_params);

    geometry_msgs::Pose2D initial_pose = c.GetCurrentPose();

    // define waypoints
    std::vector<double> initial_pos;
    initial_pos.push_back(initial_pose.x);
    initial_pos.push_back(initial_pose.y);

    std::cout << "initial pos: " << initial_pose.x << "   " << initial_pose.y << "\n";

    std::ifstream inf("../src/waypoints.txt");
    std::cout << inf << "\n";
    unsigned int waypoint_ind = 0;
    std::vector<std::vector<double> > waypoint_vector;

    if (!inf) {
        std::cout << "No goal input file, using goals specified in main.cpp" << "\n";
        std::vector<double> goal_pos;
        goal_pos.push_back(1);
        goal_pos.push_back(0);

        waypoint_vector.push_back(initial_pos);
        waypoint_vector.push_back(goal_pos);
    } else {
        std::cout << "Goal input file found, using goals specified in goals.txt" << "\n";
        waypoint_vector.push_back(initial_pos);
        while (inf) {
            std::string stringInput;
            std::vector<std::string> vector_of_strings;
            std::getline(inf, stringInput);
            std::cout << stringInput << "\n";
            if (stringInput[0] == '\0') {
                break;
            }
            split_string(stringInput, vector_of_strings);
            std::vector<double> waypoint;
            waypoint.push_back(atof(vector_of_strings[0].c_str()));
            waypoint.push_back(atof(vector_of_strings[1].c_str()));
            waypoint_vector.push_back(waypoint);
            waypoint_ind++;
        }
    }

    //
    c.SetWaypoints(waypoint_vector);
    c.GenerateWaypointSpline();
    c.GenerateVelocityWaypointSpline();

    std::cout << "Finished generating waypoint splines" << "\n";
   
    ros::Rate loop_rate(10);

    int num_reached_goal;    
    while (num_reached_goal < 10) {
        c.UpdateControllerCalculations();
        geometry_msgs::Twist twist_command = c.GetTwistCommand();
        std::cout << "Twist Command" << "\n";
        std::cout << "linear: " << twist_command.linear.x << "\n";
        std::cout << "angular: " << twist_command.angular.z << "\n";
        if (c.ReachedGoal()) {
            num_reached_goal++;
        } else {
            num_reached_goal = 0;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    //std::vector<double> nearest_waypoint;

    //nearest_waypoint = c.FindNearestWaypoint();
   
    //std::cout << nearest_waypoint[0] << "   " << nearest_waypoint[1] << "\n";

}
