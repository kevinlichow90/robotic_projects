//
//  misc_functions.h
//  arduino_serial_data_logging
//
//  Created by Kevin Chow on 10/26/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#ifndef misc_functions_h
#define misc_functions_h

bool has_any_digits(const std::string& s)
{
    return std::any_of(s.begin(), s.end(), ::isdigit);
}

bool has_any_letters(const std::string& s)
{
    return std::any_of(s.begin(), s.end(), ::isalpha);
}

#endif
