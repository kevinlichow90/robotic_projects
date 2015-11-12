//
//  main.c
//  arduino_analog_read_test
//
//  Created by Kevin Chow on 11/5/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#include <stdio.h>

int main(int argc, const char * argv[]) {
    
    
    const int num_of_analog_inputs = 3;
    double analog_input_values[num_of_analog_inputs];
    char port_name_placeholder[3];
    const char port_names[num_of_analog_inputs][3];
    
    
    for (int iii = 0; iii < num_of_analog_inputs; iii++)
    {
        snprintf(port_name_placeholder, 3, "A%d", iii);
        strcpy(port_names[iii], port_name_placeholder);
    }
    
    printf(port_names[2]);
    
    return 0;

}