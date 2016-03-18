//
//  Parameters.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef system_parameters_h
#define system_parameters_h

#include <Eigen/Dense>

struct SystemParameters {
    double mass;
    double gravity;
    Eigen::Matrix3d I;
    double arm_length;
    double minF;
    double maxF;
    double k_M;
    double k_F;
    double gamma;
};

#endif
