/*
 * DifferentialDrivePlugin.h
 *
 *  Created on: May 4, 2016
 *      Author: klchow
 */

// Uses a unicycle model to convert 2d robot velocity commands into left and right wheel velocity commands
// Contains a low level PID velocity controller

#ifndef DIFFERENTIALDRIVEPLUGIN_H_
#define DIFFERENTIALDRIVEPLUGIN_H_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <boost/bind.hpp>

namespace gazebo
{
    class DifferentialDrivePlugin : public ModelPlugin {
        private:
    	    physics::ModelPtr model;
    	    physics::JointPtr leftJoint, rightJoint;
    	    physics::LinkPtr horizontal_connector_link;
    	    math::Pose current_pose;
    	    event::ConnectionPtr updateConnection;
    	    //common::PID pid;
    	    ros::NodeHandle* rosnode_;
    	    ros::Publisher pose_pub;
    	    ros::Publisher twist_pub;
    	    ros::Subscriber sub;
    	    std::string robotNamespace;
    	    double wheel_separation_distance;
    	    double wheel_radius;
    	    double desired_velocity;
    	    double desired_angular_velocity;

    	    double current_left_joint_velocity;
    	    double current_right_joint_velocity;
    	    math::Vector3 current_robot_linear_velocity;
    	    math::Vector3 current_robot_angular_velocity;
    	    double current_robot_angle;

    	    void CommandMessageCallback(const geometry_msgs::Twist::ConstPtr &msg);

        public:
    	    DifferentialDrivePlugin();
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void Update();
            void CompareLinearToAngularVelocity();

    };

    GZ_REGISTER_MODEL_PLUGIN(DifferentialDrivePlugin)
}


#endif /* DIFFERENTIALDRIVEPLUGIN_H_ */
