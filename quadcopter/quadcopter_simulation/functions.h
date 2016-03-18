//
//  functions.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/18/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef functions_h
#define functions_h

#include <Eigen/Dense>

Eigen::Matrix3d QuaternionsToRotationMatrix(Eigen::Vector4d quaternion) {
    quaternion = quaternion/quaternion.exp(2).sum().sqrt()
    Eigen::Vector4d qahat;
    qahat(1,2) = -quaternion(4);
    qahat(1,3) = quaternion(3);
    qahat(2,3) = -quaternion(2);
    qahat(2,1) = quaternion(4);
    qahat(3,1) = -quaternion(3);
    qahat(3,2) = quaternion(2);
    
    Eigen::Matrix3d R;
    R = 
}


#endif
