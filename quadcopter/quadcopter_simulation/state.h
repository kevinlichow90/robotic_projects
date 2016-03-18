//
//  State.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef state_h
#define state_h

#include <Eigen/Dense>

struct State {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d rot;
    Eigen::Vector3d omega;
    double yaw;
    double yaw_dot;
}

#endif
