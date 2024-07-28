#include "dijkstra.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <map>
#include <tuple>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

std::tuple<vector<pair<int, int>>, vector<double>> Dijkstra::get_neighbors(pair<int, int> node) {
    vector<pair<int, int>> neighbors;
    vector<double> distances;
    vector<PlanNode> nexts = {PlanNode({-1, 0}, 1), PlanNode({-1, 1}, sqrt(2)), PlanNode({0, 1}, 1), PlanNode({1, 1}, sqrt(2)),
                          PlanNode({1, 0}, 1), PlanNode({1, -1}, sqrt(2)), PlanNode({0, -1}, 1), PlanNode({-1, -1}, sqrt(2))};
    for (auto& next : nexts) {
        pair<int, int> neighbor = {node.first + next.directions.first, node.second + next.directions.second};
        if (0 <= neighbor.first && neighbor.first < grid.size() && 0 <= neighbor.second && neighbor.second < grid[0].size()) {
            if (grid[neighbor.first][neighbor.second] == 0) {
                if (next.directions.first != 0 && next.directions.second != 0) {
                    if (grid[node.first + next.directions.first][node.second] == 0 && grid[node.first][node.second + next.directions.second] == 0) {
                        neighbors.push_back(neighbor);
                        distances.push_back(next.cost);
                    }
                } else {
                    neighbors.push_back(neighbor);
                    distances.push_back(next.cost);
                }
            }
        }
    }
    return make_tuple(neighbors,distances);
}

double Dijkstra::plan() {
    priority_queue<pair<double, pair<int, int>>, vector<pair<double, pair<int, int>>>, greater<>> priority_queue;
    priority_queue.push({0, start});
    map<pair<int, int>, double> costs;
    map<pair<int, int>, pair<int, int>> previous_nodes;
    costs[start] = 0;

    while (!priority_queue.empty()) {
        auto [current_cost, current_node] = priority_queue.top();
        priority_queue.pop();

        if (visited.find(current_node) != visited.end()) continue;

        visited.insert(current_node);
        visit_order.push_back(current_node);

        if (current_node == goal) break;

        vector<pair<int, int>> neighbors;
        vector<double> distances;
        tie(neighbors,distances) = get_neighbors(current_node);

        for (size_t i=0; i<neighbors.size();++i) {
            pair<int, int> neighbor = neighbors[i];
            double distance = distances[i];
            double cost = current_cost + distance;
            // double cost = current_cost + sqrt(pow(neighbor.first - current_node.first, 2) + pow(neighbor.second - current_node.second, 2));
            if (costs.find(neighbor) == costs.end() || cost < costs[neighbor]) {
                costs[neighbor] = cost;
                previous_nodes[neighbor] = current_node;
                priority_queue.push({cost, neighbor});
            }
        }
    }

    path.clear();
    pair<int, int> current_node = goal;
    while (current_node != start) {
        path.push_back(current_node);
        current_node = previous_nodes[current_node];
        std::cout<<"node: "<<current_node.first<<","<<current_node.second<<" ";
        printf("cost:%lf\n",costs[current_node]);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    return costs[goal];
}

void Dijkstra::visualize_grid(double cost) {
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
    VideoWriter video("dijkstra_animation.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(img_width, img_height));
    
    // Create image for each frame
    Mat img(img_height, img_width, CV_8UC3, Scalar(255, 255, 255));
    // Title and Cost
    std::string title = "Dijkstra";
    putText(img, title, Point(550, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);
    std::string cost_text = "Cost: " + std::to_string(cost);
    putText(img, cost_text, Point(500, 80), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);

    // Draw Obstacles
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
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
    for (size_t i=1;i<visit_order.size()-1;++i) {
        const auto& node = visit_order[i];
        rectangle(img, Point(node.second * cell_size, node.first * cell_size),
        Point((node.second + 1) * cell_size, (node.first + 1) * cell_size),
        Scalar(128, 128, 128), FILLED);
        
        video.write(img);
    }

    // Draw path
    for (size_t i = 1; i < path.size(); ++i) {
        Point p1(path[i - 1].second * cell_size + cell_size / 2,
                path[i - 1].first * cell_size + cell_size / 2);
        Point p2(path[i].second * cell_size + cell_size / 2,
                path[i].first * cell_size + cell_size / 2);
        line(img, p1, p2, Scalar(0, 0, 255), 2);

        video.write(img);
    }

    video.release();
    imwrite("dijkstra_result.png", img);
    imshow("Dijkstra", img);
    waitKey(0);
    destroyAllWindows();
}