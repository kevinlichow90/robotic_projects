//
//  simulation.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef simulation3d_h
#define simulation3d_h

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "trajectory_generator.h"
#include "controller.h"
#include "state.h"
#include "system_parameters.h"

class Simulation3D {
  private:
    bool real_time;
    int max_time;
    
    SystemParameters params;
    
    // initial conditions
    double tstep;
    double cstep;
    int max_iter;
    double nstep;
    double time;
    
    std::string err;
    
    State des_start;
    State des_stop;
    
    Eigen::Vector3d stop_pos;
    
    std::vector<double> x;
    std::vector<double> x0;
    std::vector<double> xtraj;
    std::vector<double> ttraj;
    
    double pos_tol;
    double vel_tol;
    
    TrajectoryGenerator trajectory_generator;
    Controller controller;
    
    
  public:
    Simulation3D();
    
    void DisplayFigure();  // skip this for now
    
    void SetInitialParameters();
    
    void RunSimulation();
    
    void PostProcessing();
    
    
};


#endif
