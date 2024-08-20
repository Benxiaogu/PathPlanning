#ifndef JPS_HPP
#define JPS_HPP

#include <iostream>
#include <vector>
#include <set>
#include <map>

using namespace std;

class JPS
{
public:
    JPS(vector<vector<int>> grid, pair<int, int> start, pair<int, int> goal, int radius, int board_size);
    double plan();
    double heuristic(pair<int,int> node);
    void visualize_grid(double cost);
    pair<int,int> Jumping(pair<int,int> node, pair<int,int> direction);
    bool findForcedNeighbor(pair<int,int> node, pair<int,int> direction);
    double calculate_cost(pair<int,int> current_node, pair<int,int> jumppoint);

private:
    vector<vector<int>> grid; 
    pair<int,int> start;
    pair<int,int> goal;
    int radius;
    int board_size;
    set<pair<int,int>> closed;
    vector<pair<int,int>> path;
    vector<pair<int,int>> visited;
};
#endif // JPS_HPP