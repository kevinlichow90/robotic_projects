//
//  main.cpp
//  arduino_serial_data_logging
//
//  Created by Kevin Chow on 10/26/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#include <iostream>
#include "serialDataCommunicator.h"
#include <string>

int main(int argc, const char * argv[]) {
    
    //testing all four motors on the car
    //collecting 25 data points for each motor to see variation in encoder readings
    
    /*
    std::cout << "Testing Motor 1" << "\n";
    std::cout << "Connect battery to motor" << "\n";
    char should_be_a;
    do
    {
        std::cout << "Press 'a' and enter when ready" << "\n";
        std::cin >> should_be_a;
    }
    while (should_be_a != 'a');
    
    serialDataCommunicator sdCommunicator1("motor1_data");
    sdCommunicator1.start();
    
    std::cout << "Testing Motor 2" << "\n";
    std::cout << "Connect battery to motor" << "\n";
    char should_be_b;
    do
    {
        std::cout << "Press 'b' and enter when ready" << "\n";
        std::cin >> should_be_b;
    }
    while (should_be_b != 'b');
    
    serialDataCommunicator sdCommunicator2("motor2_data");
    sdCommunicator2.start();
    
    std::cout << "Testing Motor 3" << "\n";
    std::cout << "Connect battery to motor" << "\n";
    char should_be_c;
    do
    {
        std::cout << "Press 'c' and enter when ready" << "\n";
        std::cin >> should_be_c;
    }
    while (should_be_c != 'c');
    
    serialDataCommunicator sdCommunicator3("motor3_data");
    sdCommunicator3.start();
    
    std::cout << "Testing Motor 4" << "\n";
    std::cout << "Connect battery to motor" << "\n";
    char should_be_d;
    do
    {
        std::cout << "Press 'd' and enter when ready" << "\n";
        std::cin >> should_be_d;
    }
    while (should_be_d != 'd');
    */
    serialDataCommunicator sdCommunicator4("motor2_data_2");
    sdCommunicator4.start();
    
    
    return 0;
}
