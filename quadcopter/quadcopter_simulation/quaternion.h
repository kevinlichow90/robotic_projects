//
//  quaternion.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/29/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef __quadcopter_simulation__quaternion__
#define __quadcopter_simulation__quaternion__

#include <stdio.h>
#include <Eigen/Dense>

class Quaternion {
  private:
    Eigen::Vector4d m_quaternion;
    
  public:
    Quaternion(Eigen::Vector4d quaternion);
    
    Quaternion();
    
    Eigen::Vector4d NormalizeQuaternion(Eigen::Vector4d quaternion);
    
    Eigen::Vector4d RotationMatrixToQuaternion(Eigen::Matrix3d rotation_matrix);
    
    Eigen::Matrix3d QuaternionToRotationMatrix(Eigen::Matrix4d quaternion);
    
    Eigen::Vector4d EulerXYZToQuaternion(Eigen::Vector3d euler_xyz);
    
    Eigen::Matrix3d QuaternionToEulerXYZ(Eigen::Vector4d quaternion);
    
    void SetQuaternion(Eigen::Vector4d quaternion);
    
    Eigen::Vector4d GetQuaternion();
    
    friend Quaternion operator*(const Quaternion &q1, const Quaternion &q2);
    // where should this function go?
    
};


#endif /* defined(__quadcopter_simulation__quaternion__) */
