//
//  controller.h
//  quadcopter_simulation
//
//  Created by Kevin Chow on 2/16/16.
//  Copyright (c) 2016 Kevin Chow. All rights reserved.
//

#ifndef controller_h
#define controller_h

#include <stdio.h>
#include "state.h"
#include "system_parameters.h"

class Controller {
    private:
        double K_p_1 = 50;
        double K_d_1 = 12.5;
        double K_p_2 = 50;
        double K_d_2 = 12.5;
        double K_p_3 = 800;
        double K_d_3 = 25;
        double K_p_phi = 50;
        double K_d_phi = 50;
        double K_p_theta = 50;
        double K_d_theta = 50;
        double K_p_psi = 50;
        double K_d_psi = 50;
    
    public:
        Controller(SystemParameters params) {
            double m = params.mass;
            double g = params.gravity;
            double I_xx = params.I(1,1);
            double I_yy = params.I(2,2);
            double I_zz = params.I(3,3);
        }
    
    int CalculateForceAndMoment {
        
    }
    
        int GetForceAndMoment(int t, State state, State des_state) {
        
        }
    
    
};


#endif /* defined(__quadcopter_simulation__controller__) */
