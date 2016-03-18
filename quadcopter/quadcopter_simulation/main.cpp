//
//  main.cpp
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#include <iostream>
#include "simulation3d.h"
#include "trajectory_generator.h"
#include <Eigen/Dense>

/*
int main()
{
    Eigen::Matrix3d m;
    m.setConstant(1.2);
    //m = Eigen::MatrixXd::Random(2,2);

    std::cout << m << std::endl << std::endl;
    std::cout << Eigen::Matrix<float,3,3>::Ones() << std::endl;
    
}
 */


int main(int argc, const char * argv[]) {
    
    TrajectoryGenerator trajectory_generator = TrajectoryGenerator();
    
    Eigen::MatrixXd waypoints(4,3);
    waypoints << 0, 0, 0,
                 1, 0, 0,
                 2, 0, 0,
                 3, 0, 0;
    
    return 0;
}
