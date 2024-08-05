/**
 * @file aStar.cpp
 * @brief Plan path using A* Algorithm
 * @author Benxiaogu:https://github.com/Benxiaogu
 * @date 2024-08-05
**/

#include "aStar.hpp"
#include <iostream>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

AStar::AStar(vector<vector<int>> grid, pair<int, int> start, pair<int, int> goal, int radius, int board_size)
            :grid(grid),start(start),goal(goal),radius(radius),board_size(board_size)
{
}

std::tuple<vector<pair<int, int>>, vector<double>> AStar::get_neighbors(pair<int, int> node) 
{
    /*
        Search the neighbors of node
        return neighbors and corresponding distances
    
    */
    vector<pair<int, int>> neighbors;
    vector<double> distances;
    vector<Next> nexts = {Next({-1, 0}, 1), Next({-1, 1}, sqrt(2)), Next({0, 1}, 1), Next({1, 1}, sqrt(2)),
                          Next({1, 0}, 1), Next({1, -1}, sqrt(2)), Next({0, -1}, 1), Next({-1, -1}, sqrt(2))};
    for (auto& next : nexts) 
    {
        pair<int, int> neighbor = {node.first + next.directions.first, node.second + next.directions.second};
        if (0 <= neighbor.first && neighbor.first < grid.size() && 0 <= neighbor.second && neighbor.second < grid[0].size()) {
            if (grid[neighbor.first][neighbor.second] == 0) {
                if (next.directions.first != 0 && next.directions.second != 0) {
                    if (grid[node.first + next.directions.first][node.second] == 0 && grid[node.first][node.second + next.directions.second] == 0) {
                        neighbors.push_back(neighbor);
                        distances.push_back(next.cost);
                    }
                } 
                else {
                    neighbors.push_back(neighbor);
                    distances.push_back(next.cost);
                }
            }
        }
    }
    return make_tuple(neighbors,distances);
}

double AStar::plan()
{
    priority_queue<pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<>> open; // Priority queue is used to  prioritise of the nodes in priority queue based on the total cost of path exploration
    map<pair<int, int>, double> g_costs;    // G(n): the cost from current node to target node
    g_costs[start] = 0;
    open.push({heuristic(start),start});
    map<pair<int, int>, pair<int, int>> parents;

    while(!open.empty())
    {
        auto [current_f, current_node] = open.top();
        open.pop();

        if(closed.find(current_node) != closed.end()) continue; // Skip it if current node has been in closed
        closed.insert(current_node);

        if(current_node == goal){   // If it reaches the goal then construct the path by backtracking
            path.clear();
            while (current_node != start) 
            {
                path.push_back(current_node);
                current_node = parents[current_node];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
            
            return g_costs[goal];
        }

        visited.push_back(current_node);
        vector<pair<int,int>> neighbors;
        vector<double> distances;
        tie(neighbors,distances) = get_neighbors(current_node);
        for (size_t i=0; i<neighbors.size();++i) 
        {
            pair<int, int> neighbor = neighbors[i];
            if(closed.find(neighbor)!=closed.end()) continue;   //Skip it if the neighbor has been in closed

            double distance = distances[i];
            double g_cost = g_costs[current_node] + distance;
            double h_cost = heuristic(neighbor);
            double f_cost = g_cost + h_cost;
            if(g_costs.find(neighbor) == g_costs.end() || g_cost<g_costs[neighbor]){
                // g_costs = open + closed, then open = g_costs - closed;
                // check whether neighbor is in g_costs without closed equal to check whether neighbor is in open, 
                // if neighbor has been in open and new g_cost of neighbor is smaller then update it
                g_costs[neighbor] = g_cost;
                parents[neighbor] = current_node;
                open.push({f_cost,neighbor});
            }
        }
    }

    return -1;
}

double AStar::heuristic(pair<int,int> node)
{
    /*
        Compute estimated cost from "node" to target node
    */
    double dx = abs(node.first - goal.first);
    double dy = abs(node.second - goal.second);

    // Manhattan distance
    // return dx+dy;

    // Euclidean Distance
    return sqrt(dx*dx+dy*dy);
}

void AStar::visualize_grid(double cost) {
    int rows = grid.size();
    int cols = grid[0].size();
    
    // Set the size of grid
    int cell_size = 20;
    int img_width = cols * cell_size;
    int img_height = rows * cell_size;

    // Create video file
    // AVI Pattern
    // VideoWriter video("dijkstra_animation.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 100, Size(img_width, img_height));
    // MP4 Pattern
    VideoWriter video("AStar_animation.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(img_width, img_height));
    
    // Create image for each frame
    Mat img(img_height, img_width, CV_8UC3, Scalar(255, 255, 255));
    // Title and Cost
    std::string title = "AStar";
    putText(img, title, Point(550, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);
    std::string cost_text = "Cost: " + std::to_string(cost);
    putText(img, cost_text, Point(500, 80), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);

    // Draw Obstacles
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            if (grid[i][j] == 1) {
                rectangle(img, Point(j * cell_size, i * cell_size),
                          Point((j + 1) * cell_size, (i + 1) * cell_size),
                          Scalar(0, 0, 0), FILLED);
            }
        }
    }

    // Draw start and goal point
    rectangle(img, Point(start.second * cell_size, start.first * cell_size),
            Point((start.second + 1) * cell_size, (start.first + 1) * cell_size),
            Scalar(0, 255, 0), FILLED);

    rectangle(img, Point(goal.second * cell_size, goal.first * cell_size),
            Point((goal.second + 1) * cell_size, (goal.first + 1) * cell_size),
            Scalar(0, 255, 255), FILLED);

    // Draw visited areas
    for (size_t i=1;i<visited.size()-1;++i) 
    {
        const auto& node = visited[i];
        rectangle(img, Point(node.second * cell_size, node.first * cell_size),
        Point((node.second + 1) * cell_size, (node.first + 1) * cell_size),
        Scalar(128, 128, 128), FILLED);
        
        video.write(img);
    }

    // Draw path
    for (size_t i = 1; i < path.size(); ++i) 
    {
        Point p1(path[i - 1].second * cell_size + cell_size / 2,
                path[i - 1].first * cell_size + cell_size / 2);
        Point p2(path[i].second * cell_size + cell_size / 2,
                path[i].first * cell_size + cell_size / 2);
        line(img, p1, p2, Scalar(0, 0, 255), 2);

        video.write(img);
    }

    video.release();
    imwrite("AStar_result.png", img);
    imshow("AStar", img);
    waitKey(0);
    destroyAllWindows();
}