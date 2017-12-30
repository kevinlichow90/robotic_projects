/*
 * SpheroDrivePlugin.cpp
 *
 *  Created on: May 4, 2016
 *      Author: klchow
 */

#include "SpheroDrivePlugin.h"
#include <iostream>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


gazebo::SpheroDrivePlugin::SpheroDrivePlugin() {
    printf("Starting the SpheroDrivePlugin\n");

}

void gazebo::SpheroDrivePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// Start up ROS
	if (!ros::isInitialized()) {
		std::string name = "sphero_drive_plugin_node";
		int argc = 0;
		ros::init(argc, NULL, name, ros::init_options::NoSigintHandler);
	}

	this->rosnode_ = new ros::NodeHandle("gazebo/sphero_drive_plugin_node");

        pose_pub = this->rosnode_->advertise<geometry_msgs::Pose>("sphero_drive_pose",1000);
        twist_pub = this->rosnode_->advertise<geometry_msgs::Twist>("sphero_drive_twist",1000);

        desired_velocity.x = 0;
        desired_velocity.y = 0;
        desired_velocity.z = 0;

	ros::V_string ros_string;
	ros::master::getNodes(ros_string);
	for (int i = 0; i < ros_string.size(); i++) {
		std::cout << ros_string[i] << "\n";
	}

	sub = this->rosnode_->subscribe<geometry_msgs::Twist>("sphero_drive_velocity_command", 1000, &gazebo::SpheroDrivePlugin::CommandMessageCallback, this);

	ros::V_string ros_string2;
	ros::this_node::getSubscribedTopics(ros_string2);
	for (int i = 0; i < ros_string2.size(); i++) {
		std::cout << ros_string2[i] << "\n";
	}

	this->model = _model;

	std::cout << "The SpheroDrivePlugin is attached to model [" << model->GetName() << "]\n";

	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&SpheroDrivePlugin::Update, this));
}

void gazebo::SpheroDrivePlugin::CommandMessageCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        desired_velocity.x = msg->linear.x;
        desired_velocity.y = msg->linear.y;
}

void gazebo::SpheroDrivePlugin::Update() {

        body_link = this->model->GetLink("body_link");
        body_link->SetWorldTwist(desired_velocity,{0,0,0});

	geometry_msgs::Pose pose_msg;
	current_pose = body_link->GetWorldCoGPose();
	pose_msg.position.x = current_pose.pos.x;
	pose_msg.position.y = current_pose.pos.y;
	pose_msg.position.z = current_pose.pos.z;
	pose_msg.orientation.x = current_pose.rot.x;
	pose_msg.orientation.y = current_pose.rot.y;
	pose_msg.orientation.z = current_pose.rot.z;
	pose_msg.orientation.w = current_pose.rot.w;
	pose_pub.publish(pose_msg);

        geometry_msgs::Twist twist_msg;
        math::Vector3 current_linear_vel = body_link->GetWorldCoGLinearVel();
        math::Vector3 current_angular_vel = body_link->GetWorldAngularVel();
        twist_msg.linear.x = current_linear_vel.x;
        twist_msg.linear.y = current_linear_vel.y;
        twist_msg.linear.z = current_linear_vel.z;
        twist_msg.angular.x = current_angular_vel.x;
        twist_msg.angular.y = current_angular_vel.y;
        twist_msg.angular.z = current_angular_vel.z;
        twist_pub.publish(twist_msg);

	ros::spinOnce();

}

