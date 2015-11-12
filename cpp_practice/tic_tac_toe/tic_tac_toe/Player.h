
//
//  Player.h
//  tic_tac_toe
//
//  Created by Kevin Chow on 11/8/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#ifndef Player_h
#define Player_h

class Player
{
private:
    char marker;
    
public:
    void Player()
    {
        SetMarker()
    }
    
    void SetMarker()
    {
        std::cout << "Enter marker type: " << "\n";
        std::cin >> marker;
        
        //add some sort of assert or exception to make sure the marker is valid (not already being used by the other player)
    }
    
    void PlaceMarker()
    {
        
    }
}


#endif
