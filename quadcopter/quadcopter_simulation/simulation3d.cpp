//
//  simulation.cpp
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#include <stdio.h>
#include "simulation3d.h"
#include "controller.h"

Simulation3D::Simulation3D() {
    real_time = true;
    max_time = 20;
    params.gravity = 9.81;
    params.mass = 0.18;
    params.I << 0.00025, 0,        2.55e-6,
                0,       0.000232, 0,
                2.55e-6, 0,        0.0003738;
    params.arm_length = 0.086;
    params.minF = 0.0;
    params.minF = 2.0*params.mass*params.gravity;
    params.k_M = 1.5e-9;
    params.k_F = 6.11e-8;
    params.gamma = params.k_M/params.k_F;
    
    
    trajectory_generator = TrajectoryGenerator();
    controller = Controller();
    
    
}

void Simulation3D::SetInitialParameters() {
    tstep = 0.01;
    cstep = 0.05;
    max_iter = max_time/cstep;
    nstep = cstep/tstep;
    time = 0;
    
    des_start = 
    
}