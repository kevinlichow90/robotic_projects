/*
 * DifferentialDrivePlugin.cpp
 *
 *  Created on: May 4, 2016
 *      Author: klchow
 */

#include "DifferentialDrivePlugin.h"
#include <iostream>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


gazebo::DifferentialDrivePlugin::DifferentialDrivePlugin() {
    printf("Starting the DifferentialDrivePlugin\n");

    // Initialize model variables
    // ***these should be on the parameter server
    wheel_separation_distance = 1.0;
    wheel_radius = 0.5;
    desired_velocity = 0;
    desired_angular_velocity = 0;
}

void gazebo::DifferentialDrivePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// Start up ROS
	if (!ros::isInitialized()) {
		std::string name = "differential_drive_plugin_node";
		int argc = 0;
		ros::init(argc, NULL, name, ros::init_options::NoSigintHandler);
	}

        //***should be private nodehandle so that all topics are published with robot name pre-appended
        this->rosnode_ = new ros::NodeHandle(_model->GetName());
	//this->rosnode_ =  new ros::NodeHandle(*private_nh,"gazebo/differential_drive_plugin_node");
	std::cout << ros::this_node::getName() << "\n";
        pose_pub = this->rosnode_->advertise<geometry_msgs::Pose>("differential_drive_pose",1000);
        twist_pub = this->rosnode_->advertise<geometry_msgs::Twist>("differential_drive_twist",1000);

	std::cout << this->rosnode_->getNamespace() << "\n";
	ros::V_string ros_string;
	ros::master::getNodes(ros_string);
	for (int i = 0; i < ros_string.size(); i++) {
		std::cout << ros_string[i] << "\n";
	}

	sub = this->rosnode_->subscribe<geometry_msgs::Twist>("differential_drive_velocity_command", 1000, &gazebo::DifferentialDrivePlugin::CommandMessageCallback, this);

	ros::V_string ros_string2;
	ros::this_node::getSubscribedTopics(ros_string2);
	for (int i = 0; i < ros_string2.size(); i++) {
		std::cout << ros_string2[i] << "\n";
	}

	this->model = _model;

	std::cout << "The DifferentialDrivePlugin is attached to model [" << model->GetName() << "]\n";

	this->leftJoint = model->GetJoint("left_wheel_joint");
	this->rightJoint = model->GetJoint("right_wheel_joint");

	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&DifferentialDrivePlugin::Update, this));
}

void gazebo::DifferentialDrivePlugin::CommandMessageCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    // body fixed axes
	//std::cout << "Received command message" << "\n";
	desired_velocity = msg->linear.x;
	desired_angular_velocity = msg->angular.z;
	//printf("Received command message\n");
}

void gazebo::DifferentialDrivePlugin::CompareLinearToAngularVelocity() {
	current_left_joint_velocity = this->leftJoint->GetVelocity(0);
	current_right_joint_velocity = this->rightJoint->GetVelocity(0);
	current_robot_linear_velocity = this->model->GetWorldLinearVel();
	current_robot_angular_velocity = this->model->GetWorldAngularVel();

	// Things to check:
	//   - Is the joint velocity the angular velocity or linear velocity?  Looks like angular velocity
	//   - Do the joint velocities match with the robot's velocity?  Linear velocity is very close.  Angular velocity is off by a bit.  Could be the wheel separation distance?  Maybe slip?
	// assuming joint velocity is angular velocity:
	double predicted_linear_velocity_1 = (wheel_radius/2)*(current_left_joint_velocity+current_right_joint_velocity)*cos(current_robot_angle);
	double predicted_angular_velocity_1 = (wheel_radius/wheel_separation_distance)*(current_right_joint_velocity-current_left_joint_velocity);
	// assuming joint velocity is linear velocity:
	double 	predicted_linear_velocity_2 = (1./2.)*(current_left_joint_velocity+current_right_joint_velocity)*cos(current_robot_angle);
	double predicted_angular_velocity_2 = (1./wheel_separation_distance)*(current_right_joint_velocity-current_left_joint_velocity);
	// comparison:
	std::cout << "predicted_linear_velocity_1: " << predicted_linear_velocity_1;
	std::cout << "predicted_linear_velocity_2: " << predicted_linear_velocity_2;
	std::cout << "current_linear_velocity: " << current_robot_linear_velocity << "\n";
	std::cout << "predicted_angular_velocity_1: " << predicted_angular_velocity_1;
	std::cout << "predicted_angular_velocity_2: " << predicted_angular_velocity_2;
	std::cout << "current_angular_velocity: " << current_robot_angular_velocity << "\n";

	//   - What are the forces and accelerations like?

	//   - Is the robot slipping?
	// v = r*w

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

        // PID wheel control
	//this->pid = gazebo::common::PID(10,0,0);
        //this->model->GetJointController()->SetVelocityPID(this->leftJoint->GetScopedName(), this->pid);
	//this->model->GetJointController()->SetVelocityPID(this->rightJoint->GetScopedName(), this->pid);
	//this->model->GetJointController()->SetVelocityTarget(this->leftJoint->GetScopedName(), desired_left_wheel_velocity);
	//this->model->GetJointController()->SetVelocityTarget(this->rightJoint->GetScopedName(), desired_right_wheel_velocity);

        // direct wheel velocity specification
        leftJoint->SetVelocity(0,desired_left_wheel_velocity);
        rightJoint->SetVelocity(0,desired_right_wheel_velocity);

	geometry_msgs::Pose pose_msg;
	this->horizontal_connector_link = this->model->GetLink("base_link");
	current_pose = this->horizontal_connector_link->GetWorldCoGPose();
	pose_msg.position.x = current_pose.pos.x;
	pose_msg.position.y = current_pose.pos.y;
	pose_msg.position.z = current_pose.pos.z;
	pose_msg.orientation.x = current_pose.rot.x;
	pose_msg.orientation.y = current_pose.rot.y;
	pose_msg.orientation.z = current_pose.rot.z;
	pose_msg.orientation.w = current_pose.rot.w;
	pose_pub.publish(pose_msg);

        geometry_msgs::Twist twist_msg;
        math::Vector3 current_linear_vel = horizontal_connector_link->GetWorldCoGLinearVel();
        math::Vector3 current_angular_vel = horizontal_connector_link->GetWorldAngularVel();
        twist_msg.linear.x = current_linear_vel.x;
        twist_msg.linear.y = current_linear_vel.y;
        twist_msg.linear.z = current_linear_vel.z;
        twist_msg.angular.x = current_angular_vel.x;
        twist_msg.angular.y = current_angular_vel.y;
        twist_msg.angular.z = current_angular_vel.z;
        twist_pub.publish(twist_msg);

	// TEMPORARY*****************
	// just used for comparing linear and angular velocity
	//convert quaternion to angle axis and then to 2D angle

	double axis[3] = {current_pose.rot.x/sqrt(1-current_pose.rot.w*current_pose.rot.w),
			           current_pose.rot.y/sqrt(1-current_pose.rot.w*current_pose.rot.w),
			           current_pose.rot.z/sqrt(1-current_pose.rot.w*current_pose.rot.w)};
	double angle = 2*acos(current_pose.rot.w)*sgn(axis[2]); // negative because that's how the signs turn out
	current_robot_angle = angle;

	ros::spinOnce();

	//CompareLinearToAngularVelocity();
}

