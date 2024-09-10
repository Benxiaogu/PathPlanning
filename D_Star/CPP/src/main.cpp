/**
 * @file main.cpp
 * @brief Example of PathPlanning
 * @author Benxiaogu:https://github.com/Benxiaogu
 * @date 2024-08-05
**/
#include "map.hpp"
#include "dstar.hpp"
#include <iostream>
#include <vector>

using namespace std;

int main() 
{
    int width = 60, height = 40, board_size = 5;
    vector<pair<int, int>> obstacles;
    for (int row = 5; row < 20; ++row) {
        obstacles.push_back({row, 25});
    }
    for (int col = 15; col < 26; ++col) {
        obstacles.push_back({20, col});
    }
    for (int row = 20; row < 35; ++row) {
        obstacles.push_back({row, 35});
    }
    for (int row = 5; row < 21; ++row) {
        obstacles.push_back({row, 45});
    }

    Map map;
    vector<vector<int>> grid = map.grid_map_user_defined(width, height, board_size, obstacles);

    pair<int, int> start = {10, 10};
    pair<int, int> goal = {30, 50};
    int radius = 1;
    
    DStar planner(grid, start, goal, radius, board_size);
    planner.run();

    return 0;
}
