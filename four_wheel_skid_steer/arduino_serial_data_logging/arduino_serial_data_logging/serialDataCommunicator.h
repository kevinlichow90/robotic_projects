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

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


class serialDataCommunicator
{
private:
    std::ofstream outf;
    std::ofstream outserial;
    std::ifstream inf;
    const int serial_data_sleep = 2;
    const int max_num_of_data = 50;
    int num_of_data = 0;
    const char serial_port[21] = "/dev/cu.usbmodem1451";
    
public:
    
    int port_fd;
    
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
    
    /*
    int InitializeCommunication() //using stdio
    {
        int fd = 0;
        struct termios options;
        
        std::cout << "Connecting to serial..." << "\n";
        //inf.open(serial_port); //if actually communicating with arduino
        
        fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1)
            return fd;
        fcntl(fd, F_SETFL, 0);
        tcgetattr(fd, &options);
        options.c_cflag &= ~(PARENB | CSTOPB);
        tcsetattr(fd, TCSANOW, &options);
        
        std::cout << "Connection successful!" << "\n";
        
        return fd;
    }
    
    void WriteDatatoFile(int fd)
    {
        char c = NULL;
        while (1)
        {
            if (read(fd, &c, 1) > 0)
            {
                std::cout << c << "\n";
            }
            
        }
    }
    */
    
    
    void InitializeCommunication() //using fstream
    {
        std::cout << "Connecting to serial..." << "\n";
        inf.open("/dev/cu.usbmodem1411");
        std::cout << inf.fail() << "\n";
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
        int fd;
        //fd = this->InitializeCommunication();
        this->InitializeCommunication();
        
        while (num_of_data < max_num_of_data)
        {
            //this->WriteDatatoFile(fd);
            this->WriteDatatoFile();
            std::this_thread::sleep_for(std::chrono::seconds(serial_data_sleep));
        }
        
    }
    
    
};


#endif
