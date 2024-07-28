#include "map.hpp"
#include "dijkstra.hpp"
#include <iostream>
#include <vector>

using namespace std;

int main() {
    Map map;
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
    for (int row = 5; row < 20; ++row) {
        obstacles.push_back({row, 45});
    }

    vector<vector<int>> grid = map.grid_map_user_defined(width, height, board_size, obstacles);

    pair<int, int> start = {10, 10};
    pair<int, int> goal = {30, 50};
    int radius = 1;

    Dijkstra planner(grid, start, goal, radius, board_size);
    double cost = planner.plan();
    planner.visualize_grid(cost);

    return 0;
}
