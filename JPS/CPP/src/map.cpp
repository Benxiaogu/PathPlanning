/**
 * @file map.cpp
 * @brief Create grid map
 * @author Benxiaogu:https://github.com/Benxiaogu
 * @date 2024-08-05
**/

#include "map.hpp"
#include <vector>
#include <set>

using namespace std;

vector<vector<int>> Map::grid_map_user_defined(int width, int height, int board_size, const vector<pair<int, int>>& obstacles) {
    grid = vector<vector<int>>(height, vector<int>(width, 0));
    this->obstacles = set<pair<int, int>>(obstacles.begin(), obstacles.end());
    
    for (int i = board_size; i < width - board_size; ++i) {
        this->obstacles.insert({board_size, i});
        this->obstacles.insert({height - board_size, i});
    }
    for (int k = board_size; k <= height - board_size; ++k) {
        this->obstacles.insert({k, board_size});
        this->obstacles.insert({k, width - board_size});
    }
    for (const auto& obstacle : this->obstacles) {
        grid[obstacle.first][obstacle.second] = 1;
    }

    return grid;
}
