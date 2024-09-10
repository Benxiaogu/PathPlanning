#ifndef MAP_HPP
#define MAP_HPP

#include <vector>
#include <set>

using namespace std;

class Map 
{
public:
    vector<vector<int>> grid;
    set<pair<int, int>> obstacles;

    Map() = default;
    vector<vector<int>> grid_map_user_defined(int width, int height, int board_size, const vector<pair<int, int>>& obstacles);
};

#endif // MAP_HPP
