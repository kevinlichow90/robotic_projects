
//
//  Grid.h
//  tic_tac_toe
//
//  Created by Kevin Chow on 11/12/15.
//  Copyright (c) 2015 Kevin Chow. All rights reserved.
//

#ifndef Grid_h
#define Grid_h

#include <string>
#include <map>
#include <iostream>


class Grid
{
private:
    std::vector<std::string> sGrid;
    
    struct Coordinate
    {
        int coordinate[2];
    };
    std::map <int, Coordinate> gridCoordinates;
    
public:
    Grid()
    {
        SetGridCoordinateMapping();
        InitializeTicTacToeGrid();
    }
    
    void InitializeTicTacToeGrid()
    {
        sGrid.push_back("- - - - - - - - - - - - -");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("- - - - - - - - - - - - -");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("- - - - - - - - - - - - -");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("|       |       |       |");
        sGrid.push_back("- - - - - - - - - - - - -");
    }
    
    void SetGridCoordinateMapping()
    {
        gridCoordinates[TOP_LEFT] = {2,4};
        gridCoordinates[TOP_MIDDLE] = {2,12};
        gridCoordinates[TOP_RIGHT] = {2,20};
        gridCoordinates[MIDDLE_LEFT] = {6,4};
        gridCoordinates[MIDDLE_MIDDLE] = {6,12};
        gridCoordinates[MIDDLE_RIGHT] = {6,20};
        gridCoordinates[BOTTOM_LEFT] = {10,4};
        gridCoordinates[BOTTOM_MIDDLE] = {10,12};
        gridCoordinates[BOTTOM_RIGHT] = {10,20};
    }
    
    void SetMarkerinGrid(int numGrid, char cMarker)
    {
        Coordinate gridCoordinate = gridCoordinates[numGrid];
        int row = gridCoordinate.coordinate[0];
        int column = gridCoordinate.coordinate[1];
        sGrid[row][column] = cMarker;
    }
    
    friend std::ostream& operator<< (std::ostream &out, Grid &grid);

};

std::ostream& operator<< (std::ostream &out, Grid &grid)
{
    for (int i = 0; i < grid.sGrid.size(); i++)
    {
        out << grid.sGrid[i] << "\n";
    }
    return out;
}

#endif
