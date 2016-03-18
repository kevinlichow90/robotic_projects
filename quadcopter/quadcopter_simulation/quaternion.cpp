//
//  quaternion.cpp
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/29/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#include "quaternion.h"

Quaternion::Quaternion(Eigen::Vector4d quaternion) : m_quaternion{quaternion}
{
}

Quaternion::Quaternion() : Quaternion::Quaternion(Eigen::Vector4d (1,0,0,0))
{
}


