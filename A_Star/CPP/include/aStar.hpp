#ifndef ASTAR_HPP
#define ASTA_HPP

#include <iostream>
#include <vector>
#include <set>
#include <map>

using namespace std;

struct Next 
{
    std::pair<int, int> directions;
    double cost;
    Next(std::pair<int, int> dir, double c) : directions(dir), cost(c) {}
};


class AStar
{
public:
    AStar(vector<vector<int>> grid, pair<int, int> start, pair<int, int> goal, int radius, int board_size);
    tuple<vector<pair<int, int>>, vector<double>> get_neighbors(pair<int, int> node);
    double plan();
    double heuristic(pair<int,int> node);
    void visualize_grid(double cost);

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
#endif // ASTAR_HPP