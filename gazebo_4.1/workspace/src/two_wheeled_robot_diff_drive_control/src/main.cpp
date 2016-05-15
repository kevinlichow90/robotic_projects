
#include "ros/ros.h"
#include "UnicycleModelControl.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>


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
    // can't instantiate the umc object in the if statement otherwise it goes out of scope
	double gain_angular_velocity;
	double gain_velocity;
    if (argc == 3) {
    	gain_angular_velocity = atof(argv[1]);
    	gain_velocity = atof(argv[2]);
    }
    else {
    	gain_angular_velocity = NULL;
    	gain_velocity = NULL;
    }

    UnicycleModelControl umc = UnicycleModelControl(gain_angular_velocity, gain_velocity);

	//UnicycleModelControl umc;
	//umc = UnicycleModelControl();

    geometry_msgs::Pose2D goal_msg;

    /*
    goal_msg.x = -1;
    goal_msg.y = 1;

    umc.SetGoal(goal_msg);
    ros::Rate loop_rate(1000);

    while (ros::ok()) {
        umc.SendControllerCommand();
        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    std::ifstream inf("src/goals.txt");
    std::cout << inf << "\n";
    double goals[100][2];
    unsigned int inside_goal_threshold_count = 0;
    unsigned int goal_ind = 0;
    if (!inf) {
    	std::cout << "No goal input file, using goals specified in main.cpp" << "\n";
    	goal_msg.x = -1;
    	goal_msg.y = 1;

    	umc.SetGoal(goal_msg);
    	ros::Rate loop_rate(1000);
        // check that robot reached the goal and stayed inside goal threshold for 0.5 seconds
    	while (inside_goal_threshold_count < 500) {
    		umc.SendControllerCommand();
    		ros::spinOnce();
    		loop_rate.sleep();
    		// checking the goal goes after sleep so that the robot has time to process the last controller command
    		if (umc.ReachedGoal() == 1) {
    		    inside_goal_threshold_count++;
    		}
    		else {
    		    inside_goal_threshold_count = 0;
    		}
    		umc.PublishCurrentRobotStatus();
    	}
    }
    else {
    	std::cout << "Goal input file found, using goals specified in goals.txt" << "\n";
    	while (inf) {
            std::string stringInput;
            std::vector<std::string> vector_of_strings;
            std::getline(inf, stringInput);
            std::cout << stringInput << "\n";
            if (stringInput[0] == '\0') {
                break;
            }
            split_string(stringInput, vector_of_strings);
            goals[goal_ind][0] = atof(vector_of_strings[0].c_str());
            goals[goal_ind][1] = atof(vector_of_strings[1].c_str());
            goal_ind++;
    	}
    	std::cout << goal_ind << "\n";
    	for (int i = 0; i < goal_ind; i++) {
    		goal_msg.x = goals[i][0];
    		goal_msg.y = goals[i][1];

    	    umc.SetGoal(goal_msg);
    	    ros::Rate loop_rate(1000);

    		while (inside_goal_threshold_count < 500) {
    			umc.SendControllerCommand();
    		    ros::spinOnce();
    			loop_rate.sleep();
    			// checking the goal goes after sleep so that the robot has time to process the last controller command
    			if (umc.ReachedGoal() == 1) {
    			    inside_goal_threshold_count++;
    			}
    			else {
    			    inside_goal_threshold_count = 0;
    			}
    			std::cout << "goal_threshold count: " << inside_goal_threshold_count << "\n";
    			umc.PublishCurrentRobotStatus();
    		}
    		std::cout << "Reached goal" << "\n";
    	}

    }


}
