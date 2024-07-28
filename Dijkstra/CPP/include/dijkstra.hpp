#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <map>
#include <tuple>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;

struct PlanNode {
    std::pair<int, int> directions;
    double cost;
    PlanNode(std::pair<int, int> dir, double c) : directions(dir), cost(c) {}
};

class Dijkstra {
public:
    Dijkstra(std::vector<std::vector<int>> grid, std::pair<int, int> start, std::pair<int, int> goal, int radius, int board_size)
        : grid(grid), start(start), goal(goal), radius(radius), board_size(board_size) {}

    std::tuple<vector<std::pair<int, int>>, vector<double>> get_neighbors(std::pair<int, int> node);
    double plan();
    void visualize_grid(double cost);

private:
    std::vector<std::vector<int>> grid;
    std::pair<int, int> start;
    std::pair<int, int> goal;
    int radius;
    int board_size;
    std::set<std::pair<int, int>> visited;
    std::vector<std::pair<int, int>> path;
    std::vector<std::pair<int, int>> visit_order;
};
#endif // DIJKSTRA_HPP
