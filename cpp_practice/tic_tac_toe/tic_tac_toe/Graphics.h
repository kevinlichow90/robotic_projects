
//
//  Graphics.h
//  tic_tac_toe
//
//  Created by Kevin Chow on 11/8/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#ifndef Graphics_h
#define Graphics_h

#include <vector>
#include <iostream>
#include "Grid.h"
#include <boost/bind.hpp>

class Graphics
{
private:
    /*
    struct GridPositions
    {
        int numGrid;
        char cMarker;
    };
     
    std::vector<GridPositions*> GridPositionArray;
    */
    
    std::vector<int> GridPositionArray1;
    std::vector<int> GridPositionArray2;
    
    Grid grid;
    
    struct WinningCombination
    {
        int combination[3];
    };
    
    std::vector<WinningCombination> winningCombinations;
    
    int countWinningCombination1;
    int countWinningCombination2;
    
public:
    Graphics()
    {
        InitializeWinningCombinations();
    }
    
    void BeginMessage()
    {
        /*
        std::cout << "- - - - - - - - - - - - -" << "\n";
        std::cout << "| o o o | o o o | o o o |" << "\n";
        std::cout << "|   o   |   o   | o     |" << "\n";
        std::cout << "|   o   | o o o | o o o |" << "\n";
        std::cout << "- - - - - - - - - - - - -" << "\n";
        std::cout << "| o o o |   o   | o o o |" << "\n";
        std::cout << "|   o   | o o o | o     |" << "\n";
        std::cout << "|   o   | o   o | o o o |" << "\n";
        std::cout << "- - - - - - - - - - - - -" << "\n";
        std::cout << "| o o o | o o o | o o o |" << "\n";
        std::cout << "|   o   | o   o | o o o |" << "\n";
        std::cout << "|   o   | o o o | o o o |" << "\n";
        std::cout << "- - - - - - - - - - - - -" << "\n";
        */
        
        std::cout << "- - - - - - - - - - - - -" << "\n";
        std::cout << "| - - - | - - - | - - - |" << "\n";
        std::cout << "|   |   |   |   | |     |" << "\n";
        std::cout << "|   |   | - - - | - - - |" << "\n";
        std::cout << "- - - - - - - - - - - - -" << "\n";
        std::cout << "| - - - |   -   | - - - |" << "\n";
        std::cout << "|   |   | | - | | |     |" << "\n";
        std::cout << "|   |   | |   | | - - - |" << "\n";
        std::cout << "- - - - - - - - - - - - -" << "\n";
        std::cout << "| - - - | - - - | - - - |" << "\n";
        std::cout << "|   |   | |   | | | - - |" << "\n";
        std::cout << "|   |   | - - - | - - - |" << "\n";
        std::cout << "- - - - - - - - - - - - -" << "\n";
    }
    
    void InitializeWinningCombinations()
    {
        WinningCombination wc1 = {0, 1, 2};
        WinningCombination wc2 = {3, 4, 5};
        WinningCombination wc3 = {6, 7, 8};
        WinningCombination wc4 = {0, 3, 6};
        WinningCombination wc5 = {1, 4, 7};
        WinningCombination wc6 = {2, 5, 8};
        WinningCombination wc7 = {0, 4, 9};
        WinningCombination wc8 = {3, 5, 7};
        winningCombinations.push_back(wc1);
        winningCombinations.push_back(wc2);
        winningCombinations.push_back(wc3);
        winningCombinations.push_back(wc4);
        winningCombinations.push_back(wc5);
        winningCombinations.push_back(wc6);
        winningCombinations.push_back(wc7);
        winningCombinations.push_back(wc8);
    }
    
    void DisplayGrid()
    {
        std::cout << grid << "\n";
    }
    
    int SetNewGridPostion1(int numGrid, char cMarker) //possibly move to player class
    {
        if (std::find(GridPositionArray1.begin(), GridPositionArray1.end(), numGrid) != GridPositionArray1.end())
        {
            return 0;
        }
        else if (std::find(GridPositionArray2.begin(), GridPositionArray2.end(), numGrid) != GridPositionArray2.end())
        {
            return 0;
        }
        GridPositionArray1.push_back(numGrid);
        grid.SetMarkerinGrid(numGrid, cMarker);
        return 1;
        //could have separate grid position arrays for each player, but then you'd have to create a separate function
        //could use find if and just use a single grid position array
    }
    
    int SetNewGridPostion2(int numGrid, char cMarker)
    {
        if (std::find(GridPositionArray1.begin(), GridPositionArray1.end(), numGrid) != GridPositionArray1.end())
        {
            return 0;
        }
        else if (std::find(GridPositionArray2.begin(), GridPositionArray2.end(), numGrid) != GridPositionArray2.end())
        {
            return 0;
        }
        GridPositionArray2.push_back(numGrid);
        grid.SetMarkerinGrid(numGrid, cMarker);
        return 1;
        //could have separate grid position arrays for each player, but then you'd have to create a separate function
        //could use find if and just use a single grid position array
    }
    
    
    bool CheckWin() //possibly move to Player class
    {
        for (int i = 0; i < winningCombinations.size(); i++)
        {
            countWinningCombination1 = 0;
            for (int j = 0; j < GridPositionArray1.size(); j++)
            {
                if (std::find(std::begin(winningCombinations[i].combination), std::end(winningCombinations[i].combination), GridPositionArray1[j]) != std::end(winningCombinations[i].combination)) //hardcoded size of
                    countWinningCombination1++;
            }
            if (countWinningCombination1 == 3)
            {
                EndMessage("Player 1");
                return true;
            }
            
            countWinningCombination2 = 0;
            for (int j = 0; j < GridPositionArray2.size(); j++)
            {
                if (std::find(std::begin(winningCombinations[i].combination), std::end(winningCombinations[i].combination), GridPositionArray2[j]) != std::end(winningCombinations[i].combination)) //hardcoded size of
                    countWinningCombination2++;
            }
            if (countWinningCombination2 == 3)
            {
                EndMessage("Player 2");
                return true;
            }

        }
        return false;
    }
     
    
    void EndMessage(std::string whoWon)
    {
        std::cout << whoWon << " wins!!!!!!" << "\n";
    }
};

#endif
