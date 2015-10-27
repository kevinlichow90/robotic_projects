//
//  serialDataCommunicator.h
//  arduino_serial_data_logging
//
//  Created by Kevin Chow on 10/26/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#ifndef serialDataCommunicator_h
#define serialDataCommunicator_h

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include "misc_functions.h"


class serialDataCommunicator
{
private:
    std::ofstream outf;
    std::ifstream inf;
    const int serial_data_sleep = 2;
    const int max_num_of_data = 25;
    int num_of_data = 0;
    
public:
    serialDataCommunicator(const char *strName)
        : outf(strName)
    {
        if (!outf)
        {
            std::cerr << strName << " could not be opened for writing." << "\n";
            exit(1); //use exception instead of exit?
        }
    }
    
    serialDataCommunicator()
        : outf("test")
    {
        if (!outf)
        {
            std::cerr << "test could not be opened for writing." << "\n";
            exit(1); //use exception instead of exit?
        }
    }
    
    void InitializeCommunication()
    {
        std::cout << "Connecting to serial..." << "\n";
        inf.open("/dev/cu.usbmodem1451");
        std::cout << "Connection successful!" << "\n";
    }
    
    
    void WriteDatatoFile()
    {
        std::string strInput;
        getline(inf, strInput);
        //strInput.erase(std::remove_if(strInput.begin(), strInput.end(), isspace)); // this was added for testing
        std::cout << "Raw data from serial: " << strInput << "\n";
        if (has_any_letters(strInput))
            std::cout << strInput << " has letters" << "\n";
        
        if (has_any_digits(strInput))
            std::cout << strInput << " has digits" << "\n";
        
        if (has_any_digits(strInput) || has_any_letters(strInput))
        {
            outf << strInput << "\n";
            std::cout << "Wrote " << strInput << " to file" << "\n";
            ++num_of_data;
        }
    }
    
    void start()
    {
        this->InitializeCommunication();
        
        while (num_of_data < max_num_of_data)
        {
            this->WriteDatatoFile();
            std::this_thread::sleep_for(std::chrono::seconds(serial_data_sleep));
        }
    }
    
    
};


#endif
