/*
 * DifferentialDrivePlugin.h
 *
 *  Created on: May 4, 2016
 *      Author: klchow
 */

#ifndef DIFFERENTIALDRIVEPLUGIN_H_
#define DIFFERENTIALDRIVEPLUGIN_H_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/math/Pose.hh"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
//#include "gazebo/util/system.hh"
//#include "gazebo/common/Plugin.hh"
//#include "gazebo/common/CommonTypes.hh"
#include <boost/bind.hpp>
#include <memory>

namespace gazebo
{
    class DifferentialDrivePlugin : public ModelPlugin {
        private:
            //void VelocityCommandCallback(ConstPosePtr);
    	    physics::ModelPtr model;
    	    physics::JointPtr leftJoint, rightJoint;
    	    physics::LinkPtr horizontal_connector_link;
    	    math::Pose current_pose;
    	    event::ConnectionPtr updateConnection;
    	    common::PID pid;
    	    transport::NodePtr node;
    	    transport::SubscriberPtr commandSub;
    	    ros::CallbackQueue rosQueue;
    	    ros::NodeHandle* rosnode_;
    	    ros::Publisher pose_pub;
    	    ros::Subscriber sub;
    	    std::string robotNamespace;
    	    double wheel_separation_distance;
    	    double wheel_radius;
    	    double desired_velocity;
    	    double desired_angular_velocity;

    	    //boost::shared_ptr<ros::NodeHandle> rosNode;
    	    void CommandMessageCallback(const geometry_msgs::Twist::ConstPtr &msg);
    	    //void CommandMessageCallback(const std_msgs::Float32::ConstPtr &msg);
    	    //void CommandMessageCallback(ConstPosePtr &msg);

        public:
    	    DifferentialDrivePlugin();
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void Update();

    };

    GZ_REGISTER_MODEL_PLUGIN(DifferentialDrivePlugin)
}


#endif /* DIFFERENTIALDRIVEPLUGIN_H_ */
