//
//  main.cpp
//  tic_tac_toe
//
//  Created by Kevin Chow on 11/8/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#include <iostream>
#include <map>

enum GridPosition
{
    TOP_LEFT,
    TOP_MIDDLE,
    TOP_RIGHT,
    MIDDLE_LEFT,
    MIDDLE_MIDDLE,
    MIDDLE_RIGHT,
    BOTTOM_LEFT,
    BOTTOM_MIDDLE,
    BOTTOM_RIGHT
};
// is there a better way to organize this?
#include "Player.h"
#include "Graphics.h"

std::map<std::string, GridPosition> sGPMap;

void setSGPMap()
{
    sGPMap["TOP_LEFT"] = TOP_LEFT;
    sGPMap["TOP_MIDDLE"] = TOP_MIDDLE;
    sGPMap["TOP_RIGHT"] = TOP_RIGHT;
    sGPMap["MIDDLE_LEFT"] = MIDDLE_LEFT;
    sGPMap["MIDDLE_MIDDLE"] = MIDDLE_MIDDLE;
    sGPMap["MIDDLE_RIGHT"] = MIDDLE_RIGHT;
    sGPMap["BOTTOM_LEFT"] = BOTTOM_LEFT;
    sGPMap["BOTTOM_MIDDLE"] = BOTTOM_MIDDLE;
    sGPMap["BOTTOM_RIGHT"] = BOTTOM_RIGHT;
    
}


int main(int argc, const char * argv[]) {
    
    Graphics graphics;
    graphics.BeginMessage();
    Player pPlayer1(1);
    Player pPlayer2(2);
    while (pPlayer1.GetMarker() == pPlayer2.GetMarker())
    {
        std::cout << pPlayer1.GetMarker() << "\n";
        std::cout << pPlayer2.GetMarker() << "\n";
        std::cout << "Both players have the same marker.  Please change the markers." << "\n";
        pPlayer1.SetMarker();
        pPlayer2.SetMarker();
    }
    
    setSGPMap();
    bool gameOver;
    std::string player1Move;
    std::string player2Move;
    while (true)
    {
        std::cout << "Player 1, please enter a position (TOP_LEFT, TOP_MIDDLE, TOP_RIGHT, MIDDLE_LEFT, MIDDLE_MIDDLE, MIDDLE_RIGHT, BOTTOM_LEFT, BOTTOM_MIDDLE, BOTTOM_RIGHT): ";
        std::cin >> player1Move;
        //add error checking to make sure that they enter a valid position
        graphics.SetNewGridPostion1(sGPMap[player1Move], pPlayer1.GetMarker());
        graphics.DisplayGrid();
        if (graphics.CheckWin())
        {
            break;
        }
        
        
        std::cout << "Player 2, please enter a position (TOP_LEFT, TOP_MIDDLE, TOP_RIGHT, MIDDLE_LEFT, MIDDLE_MIDDLE, MIDDLE_RIGHT, BOTTOM_LEFT, BOTTOM_MIDDLE, BOTTOM_RIGHT): ";
        std::cin >> player2Move;
        graphics.SetNewGridPostion2(sGPMap[player2Move], pPlayer2.GetMarker());
        graphics.DisplayGrid();
        if (graphics.CheckWin())
        {
            break;
        }
    }
    return 0;
}
