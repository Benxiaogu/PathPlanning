/**
 * @file jps.cpp
 * @brief Plan path using Jump Point Search(JPS) Algorithm
 *
 * @author Benxiaogu
 * https://blog.csdn.net/weixin_51995147?type=blog
 * https://github.com/Benxiaogu
 *
 * @date 2024-08-19
 */

#include "jps.hpp"
#include <iostream>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

JPS::JPS(vector<vector<int>> grid, pair<int, int> start, pair<int, int> goal, int radius, int board_size)
            :grid(grid),start(start),goal(goal),radius(radius),board_size(board_size)
{
}

double JPS::plan()
{
    priority_queue<pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<>> open; // Priority queue is used to  prioritise of the nodes in priority queue based on the total cost of path exploration
    map<pair<int, int>, double> g_costs;    // G(n): the cost from current node to target node
    g_costs[start] = 0;
    open.push({heuristic(start),start});
    map<pair<int, int>, pair<int, int>> parents;

    vector<pair<int, int>> nexts = {{-1, 0},{0, 1},{0, -1},{1, 0},{-1, 1},{1, -1},{1, 1},{-1, -1}};

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

        for (auto& next:nexts)
        {
            pair<int,int> jumppoint = Jumping(current_node,next);
            if(jumppoint.first>0){
                if(closed.find(jumppoint)==closed.end()){
                    double h = heuristic(jumppoint);
                    double g = g_costs[current_node] + calculate_cost(current_node,jumppoint);
                    double f = g + h;
                    g_costs[jumppoint] = g;
                    parents[jumppoint] = current_node;
                    open.push({f,jumppoint});
                    visited.push_back(jumppoint);
                    if(jumppoint == goal){
                        break;
                    }

                }

            }
        }
    }

    return -1;
}

pair<int,int> JPS::Jumping(pair<int,int> node, pair<int,int> direction)
{
    /*
        * Parameters: 
                node: explore if there is jump point based on the node
                direction: direction of exploration

        * Return:
                pair<int,int>: return jump point if it exists, otherwise (-1,-1)
    */
    pair<int,int> new_node = {node.first + direction.first, node.second + direction.second};
    if (grid[new_node.first][new_node.second]==1){
        return {-1,-1};
    }
    if (new_node==goal){
        return new_node;
    }
    if (findForcedNeighbor(new_node,direction)){
        return new_node;
    }

    if (direction.first!=0 && direction.second!=0){
        pair<int,int> y_dir = {direction.first,0};
        pair<int,int> x_dir = {0,direction.second};
        if (Jumping(new_node,x_dir).first!=-1 || Jumping(new_node,y_dir).first!=-1){
            return new_node;
        }
    }

    return Jumping(new_node,direction);
}

bool JPS::findForcedNeighbor(pair<int,int> node, pair<int,int> direction)
{
    /*
        * Parameters: 
                node: determine if the node has a forced neighbor
                direction: direction for determining whether there is a forced neighbor
        * Return: 
                bool: return true if a forced neighbor exists otherwise false
    */
    // vertical
    if(direction.first!=0 && direction.second==0){
        if((grid[node.first][node.second+1]!=0 && grid[node.first+direction.first][node.second+1]==0) ||
            (grid[node.first][node.second-1]!=0 && grid[node.first+direction.first][node.second-1]==0)){
                return true;
            }
    }

    //horizontal
    if(direction.first==0 && direction.second!=0){
        if((grid[node.first+1][node.second]!=0 && grid[node.first+1][node.second+direction.second]==0) ||
            (grid[node.first-1][node.second]!=0 && grid[node.first-1][node.second+direction.second]==0)){
                return true;
            }
    }

    // diagonal
    if(direction.first!=0 && direction.second!=0){
        if((grid[node.first-direction.first][node.second]!=0 && grid[node.first-direction.first][node.second+direction.second]==0) ||
            (grid[node.first][node.second-direction.second]!=0 && grid[node.first+direction.first][node.second-direction.second]==0)){
                return true;
            }
    }

    return false;
}

double JPS::calculate_cost(pair<int,int> current_node, pair<int,int> jumppoint)
{
    /*
        * Parameters:
                current_node: the current node on the path to goal
                jumppoint: the jump point searched by the current node
        * Return:
                the distance from current node to jumppoint (the value of g)
    */
    double dy = abs(current_node.first - jumppoint.first);
    double dx = abs(current_node.second - jumppoint.second);

    return sqrt(dx*dx+dy*dy);
}

double JPS::heuristic(pair<int,int> node)
{
    /*
        Compute estimated cost from "node" to target node
    */
    double dy = abs(node.first - goal.first);
    double dx = abs(node.second - goal.second);

    // Manhattan distance
    // return dx+dy;

    // Euclidean Distance
    return sqrt(dx*dx+dy*dy);
}

void JPS::visualize_grid(double cost) {
    int rows = grid.size();
    int cols = grid[0].size();
    
    // Set the size of grid
    int cell_size = 20;
    int img_width = cols * cell_size;
    int img_height = rows * cell_size;

    // Create video file
    // AVI Pattern
    // VideoWriter video("JPS_animation.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 100, Size(img_width, img_height));
    // MP4 Pattern
    VideoWriter video("JPS_animation.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(img_width, img_height));
    
    // Create image for each frame
    Mat img(img_height, img_width, CV_8UC3, Scalar(255, 255, 255));
    // Title and Cost
    std::string title = "JPS";
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
    imwrite("JPS_result.png", img);
    imshow("JPS", img);
    waitKey(0);
    destroyAllWindows();
}