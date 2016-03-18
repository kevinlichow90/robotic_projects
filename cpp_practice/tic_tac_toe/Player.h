
//
//  Player.h
//  tic_tac_toe
//
//  Created by Kevin Chow on 11/8/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#ifndef Player_h
#define Player_h

#include <iomanip>
#include <limits>

class Player
{
private:
    char cMarker;
    int numPlayer;
    
public:
    Player(int num)
    {
        numPlayer = num;
        SetMarker();
    }
    
    void SetMarker()
    {
        std::cout << "Player " << numPlayer << " enter a marker (single char): ";
        
        std::cin >> cMarker;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n'); //flush input stream after the first one
        //add some sort of assert or exception to make sure the marker is valid (not already being used by the other player)
    }
    
    char GetMarker()
    {
        return cMarker;
    }
    
    void PlaceMarker()
    {
        
    }
};


#endif
