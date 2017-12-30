/*
 * SpheroDrivePlugin.h
 *
 *  Created on: May 4, 2016
 *      Author: klchow
 */

// Contains a low level PID velocity controller

#ifndef SPHERODRIVEPLUGIN_H_
#define SPHERODRIVEPLUGIN_H_

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector3.hh"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <boost/bind.hpp>
#include <memory>

namespace gazebo
{
    class SpheroDrivePlugin : public ModelPlugin {
        private:
    	    physics::ModelPtr model;
    	    physics::LinkPtr body_link;
    	    math::Pose current_pose;
    	    event::ConnectionPtr updateConnection;
    	    ros::NodeHandle* rosnode_;
    	    ros::Publisher pose_pub;
    	    ros::Publisher twist_pub;
    	    ros::Subscriber sub;
    	    std::string robotNamespace;
    	    math::Vector3 desired_velocity;

    	    void CommandMessageCallback(const geometry_msgs::Twist::ConstPtr &msg);

        public:
    	    SpheroDrivePlugin();
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void Update();

    };

    GZ_REGISTER_MODEL_PLUGIN(SpheroDrivePlugin);
}


#endif /* SPHERODRIVEPLUGIN_H_ */
