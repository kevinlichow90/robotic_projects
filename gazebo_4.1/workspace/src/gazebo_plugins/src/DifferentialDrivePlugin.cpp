/*
 * DifferentialDrivePlugin.cpp
 *
 *  Created on: May 4, 2016
 *      Author: klchow
 */

#include "DifferentialDrivePlugin.h"
#include <iostream>


gazebo::DifferentialDrivePlugin::DifferentialDrivePlugin() {
    printf("Starting the DifferentialDrivePlugin\n");

    // Initialize model variables
    wheel_separation_distance = 1.0;
    wheel_radius = 0.5;
    this->desired_velocity = 0;
    this->desired_angular_velocity = 0;
}

void gazebo::DifferentialDrivePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// Start up ROS
	if (!ros::isInitialized()) {
		std::string name = "differential_drive_plugin_node";
		int argc = 0;
		ros::init(argc, NULL, name, ros::init_options::NoSigintHandler);
	}

	//this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString();
	this->rosnode_ = new ros::NodeHandle("gazebo/differential_drive_plugin_node");
	std::cout << ros::this_node::getName() << "\n";
    pose_pub = this->rosnode_->advertise<geometry_msgs::Pose>("differential_drive_pose",1000);
    //this->rosnode_->subscribe<geometry_msgs::Twist>("/differential_drive_velocity_command", 1000, &DifferentialDrivePlugin::CommandMessageCallback, this);
	//ros::NodeHandle rosnode_;
	//this->rosNode.reset(new ros::NodeHandle("test"));

	std::cout << this->rosnode_->getNamespace() << "\n";
	ros::V_string ros_string;
	ros::master::getNodes(ros_string);
	for (int i = 0; i < ros_string.size(); i++) {
		std::cout << ros_string[i] << "\n";
	}

	//ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>("/gazebo/differential_drive_plugin_node/differential_drive_velocity_command", 1000, boost::bind(&DifferentialDrivePlugin::CommandMessageCallback, this, _1), ros::VoidPtr(), this->rosnode_->getCallbackQueue());
	//this->rosnode_->subscribe(so);
	sub = this->rosnode_->subscribe<geometry_msgs::Twist>("differential_drive_velocity_command", 1000, &gazebo::DifferentialDrivePlugin::CommandMessageCallback, this);
	//this->rosNode->subscribe<geometry_msgs::Twist>("/gazebo/differential_drive_velocity_comand", 1000, &DifferentialDrivePlugin::CommandMessageCallback, this);

	ros::V_string ros_string2;
	ros::this_node::getSubscribedTopics(ros_string2);
	for (int i = 0; i < ros_string2.size(); i++) {
		std::cout << ros_string2[i] << "\n";
	}

	this->model = _model;

	//this->node = transport::NodePtr(new gazebo::transport::Node());
	//this->node->Init(this->model->GetWorld()->GetName());
    //this->commandSub = this->node->Subscribe(std::string("~/")+this->model->GetName()+"/vel_cmd", &DifferentialDrivePlugin::CommandMessageCallback, this);

	std::cout << "The DifferentialDrivePlugin is attached to model [" << model->GetName() << "]\n";

	this->leftJoint = model->GetJoint("left_wheel_joint");
	this->rightJoint = model->GetJoint("right_wheel_joint");

	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&DifferentialDrivePlugin::Update, this));
}

void gazebo::DifferentialDrivePlugin::CommandMessageCallback(const geometry_msgs::Twist::ConstPtr &msg) {
//void gazebo::DifferentialDrivePlugin::CommandMessageCallback(const std_msgs::Float32::ConstPtr &msg) {
	//void gazebo::DifferentialDrivePlugin::CommandMessageCallback(ConstPosePtr &msg) {
    // body fixed axes
	//std::cout << "Received command message" << "\n";
	this->desired_velocity = msg->linear.x;
	this->desired_angular_velocity = msg->angular.z;
	//printf("Received command message\n");
}

void gazebo::DifferentialDrivePlugin::Update() {

	double desired_left_wheel_velocity = (2*desired_velocity-this->desired_angular_velocity*wheel_separation_distance)/(2*wheel_radius);
    double desired_right_wheel_velocity = (2*desired_velocity+this->desired_angular_velocity*wheel_separation_distance)/(2*wheel_radius);
	//double desired_left_wheel_velocity = 0;
	//double desired_right_wheel_velocity = 0;

	this->leftJoint = model->GetJoint("left_wheel_joint");
	this->rightJoint = model->GetJoint("right_wheel_joint");
	//std::cout << this->leftJoint->GetName() << "\n";
	//std::cout << this->rightJoint->GetName() << "\n";
	//std::cout << "Setting the left wheel velocity to " << desired_left_wheel_velocity << " and the right wheel velocity to " << desired_right_wheel_velocity << "\n";

	this->pid = gazebo::common::PID(10,0,0);

    this->model->GetJointController()->SetVelocityPID(this->leftJoint->GetScopedName(), this->pid);
	this->model->GetJointController()->SetVelocityPID(this->rightJoint->GetScopedName(), this->pid);

	this->model->GetJointController()->SetVelocityTarget(this->leftJoint->GetScopedName(), desired_left_wheel_velocity);
	this->model->GetJointController()->SetVelocityTarget(this->rightJoint->GetScopedName(), desired_right_wheel_velocity);

	//std::cout << this->leftJoint->GetVelocity(0) << "\n";
	//std::cout << this->rightJoint->GetVelocity(0) << "\n";

	geometry_msgs::Pose msg;
	this->horizontal_connector_link = this->model->GetLink("base_footprint");
	current_pose = this->horizontal_connector_link->GetWorldCoGPose();
	msg.position.x = current_pose.pos.x;
	msg.position.y = current_pose.pos.y;
	msg.position.z = current_pose.pos.z;
	msg.orientation.x = current_pose.rot.x;
	msg.orientation.y = current_pose.rot.y;
	msg.orientation.z = current_pose.rot.z;
	msg.orientation.w = current_pose.rot.w;
	pose_pub.publish(msg);

	ros::spinOnce();
}

