//
//  quadcopter_equations_of_motions.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef quadcopter_equations_of_motions_h
#define quadcopter_equations_of_motions_h

#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include "state.h"
#include "system_parameters.h"
#include "trajectory_generator.h"
#include "controller.h"

class QuadcopterEquationsOfMotion {
    private:
        double m_t;
        State &m_state;
        TrajectoryGenerator &m_trajectory_generator;
        Controller &m_controller;
        SystemParameters m_params;
    
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
        Eigen::Vector4d prop_thrusts;
        Eigen::Vector4d prop_thrusts_clamped;
    
        double F;
        Eigen::Vector3d M;
    
        State sdot_state;
    
        std::vector<double> ForceMomentVector;
    
        double x;
        double y;
        double z;
        double xdot;
        double ydot;
        double zdot;
        double qW;
        double qX;
        double qY;
        double qZ;
        double p;
        double q;
        double r;
    
        Eigen::Vector4d quat;
    
    
    public:
    QuadcopterEquationsOfMotion(double t, State &state, TrajectoryGenerator &trajectory_generator, Controller &controller, SystemParameters params) : m_t{t}, m_state{state}, m_trajectory_generator{trajectory_generator}, m_controller{controller}, m_params(params) { // why does this have to be parenthesis? uniform initialization
        }
    
        State CalculateSDot() {
            
            ForceMomentVector = m_controller.GetFM();
            
            // First apply force limits
            
            A = Eigen::MatrixXd(4,3);
            A << 0.25, 0, -0.5/m_params.arm_length, 0.25/m_params.gamma,
                 0.25, 0.5/m_params.arm_length, 0, -0.25/m_params.gamma,
                 0.25, 0, 0.5/m_params.arm_length, 0.25/m_params.gamma,
                 0.25, -0.5/m_params.arm_length, 0, -0.25/m_params.gamma;
            
            prop_thrusts = A*Eigen::Vector3d(ForceMomentVector[0], ForceMomentVector[1], ForceMomentVector[2]);
            
            for (int F_index = 0; F_index < prop_thrusts.size(); F_index++) {
                if (prop_thrusts[F_index] < (m_params.minF/4)) {
                    prop_thrusts_clamped[F_index] = m_params.minF/4;
                }
                else if (prop_thrusts[F_index] > (m_params.maxF/4)) {
                    prop_thrusts_clamped[F_index] = m_params.maxF/4;
                }
                else {
                    prop_thrusts_clamped[F_index] = prop_thrusts[F_index];
                }
            }
            
            B = Eigen::MatrixXd(3,4);
            B << 1, 1, 1, 1,
                0, m_params.arm_length, 0, -m_params.arm_length,
                -m_params.arm_length, 0, m_params.arm_length, 0;
            
            F = B.row(1)*prop_thrusts_clamped;
            M = B.block<2,4>(1,0)*prop_thrusts_clamped;
            
            
            
        }
    
    
    
};

#endif /* defined(__quadcopter_simulation__quadcopter_equations_of_motions__) */
