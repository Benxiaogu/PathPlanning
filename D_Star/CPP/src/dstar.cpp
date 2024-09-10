#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <set>
#include <memory>
#include <opencv2/opencv.hpp>
#include "dstar.hpp"

using namespace std;
using namespace cv;

DStar::DStar(vector<vector<int>> grid, pair<int, int> start, pair<int, int> goal, int radius, int board_size)
            :grid(grid),radius(radius),board_size(board_size)
{
    startNode = new DNode{start, {-1,-1}, "NEW", 10000, 10000};
    goalNode = new DNode{goal, {-1,-1}, "NEW", 0, 10000};

    for (int i = 0; i < grid.size(); ++i) {
        for (int j = 0; j < grid[0].size(); ++j) {
            closed[{i, j}] = new DNode{{i, j}, {-1, -1}, "NEW", 10000, 10000};
        }
    }
    closed[start] = startNode;
    closed[goal] = goalNode;
    insert(goalNode, 0);
}

void DStar::run()
{
    double cost = plan();
    std::cout << "Total path cost: " << cost << std::endl;

    // Set callback function of mouse-event
    cv::namedWindow("DStar", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("DStar", onMouse, this);

    visualize(cost);

    // Exit when you press any key.
    waitKey(0);
    destroyAllWindows();

    // Clear the openlist.
    openlist.clear();
    // Delete nodes in closed to release memory.
    delete startNode;
    delete goalNode;
    for (int i = 0; i < grid.size(); i++)
        for (int j = 0; j < grid[0].size(); j++)
        delete closed[{i,j}];
}

double DStar::plan()
{
    while (true) 
    {
        double k_min = process_state();
        if (startNode->t == "CLOSED" || k_min == -1) {
            break;
        }
    }
    visited.clear();

    double cost = extractPath();

    return cost;
}

double DStar::process_state() {
    if (openlist.empty()) {
        printf("openlist is empty!\n");
        return -1;
    }
    DNode* node = min_state();
    visited.push_back(node->position);

    double k_old = node->k;
    delete_node(node);

    auto [neighbors, distances] = get_neighbors(node);
    if (k_old < node->h) {
        // "RAISE State"
        // When the current node is affected by the added obstacle.
        // (e.g., the parent of the current node is added obstacle)
        for (size_t i = 0; i < neighbors.size(); ++i) {
            DNode* neighbor = neighbors[i];
            if (neighbor->h <= k_old && node->h > neighbor->h + distances[i]) {
                node->parent = neighbor->position;
                node->h = neighbor->h + distances[i];
                // Expand in the direction of the smaller value of h, the cost from the current node to the goal node, 
                // and update the parent node of the current node to update the path.
            }
        }
    }

    if (k_old == node->h) {
        // "LOWER State"
        // The current node have not yet been affected by the added obstacle.
        for (size_t i = 0; i < neighbors.size(); ++i) {
            DNode* neighbor = neighbors[i];
            if (neighbor->t == "NEW" || (neighbor->parent == node->position && neighbor->h != node->h + distances[i])
                || (neighbor->parent != node->position && neighbor->h > node->h + distances[i])) {
                neighbor->parent = node->position;
                insert(neighbor, node->h + distances[i]);
            }
        }
    }
    else {
        for (size_t i = 0; i < neighbors.size(); ++i) {
            DNode* neighbor = neighbors[i];
            if (neighbor->t == "NEW" || (neighbor->parent == node->position && neighbor->h != node->h + distances[i])) {
                neighbor->parent = node->position;
                insert(neighbor, node->h + distances[i]);
            }
            else{
                if (neighbor->parent != node->position && neighbor->h > node->h + distances[i]) 
                {
                    insert(node, node->h);
                }
                else{
                    if (neighbor->parent != node->position && node->h > neighbor->h + distances[i] 
                        && neighbor->t == "CLOSED" && neighbor->h > k_old) 
                    {
                        insert(neighbor, neighbor->h);
                    }
                }
            }
        }
    }
    printf("process_state end\n");

    return get_kmin();
}

DNode* DStar::min_state() 
{
    return openlist.begin()->second;
}

double DStar::get_kmin()
{
    return openlist.begin()->first;
}

tuple<vector<DNode*>, vector<double>> DStar::get_neighbors(DNode* current_node) 
{
    pair<int,int> node = current_node->position;
    std::vector<DNode*> neighbors;
    std::vector<double> distances;
    std::vector<Neighbor> nexts = {Neighbor({-1, 0}, 1), Neighbor({0, 1}, 1), Neighbor({0, -1}, 1), Neighbor({1, 0}, 1),
                                    Neighbor({-1, 1}, std::sqrt(2)), Neighbor({1, 1}, std::sqrt(2)), Neighbor({1, -1}, std::sqrt(2)), Neighbor({-1, -1}, std::sqrt(2))};
    for (auto& next : nexts) {
        pair<int, int> neighbor = {node.first + next.direction.first, node.second + next.direction.second};
        DNode* neighborNode = closed[neighbor];
        if (!isCollision(node, neighbor)) {
            neighbors.push_back(neighborNode);
            distances.push_back(next.cost);
        }
    }

    return {neighbors,distances};
}

void DStar::insert(DNode* node, double h_new) {
    if (node->t == "NEW") node->k = h_new;
    else if (node->t == "OPEN") node->k = std::min(node->k, h_new);
    else if (node->t == "CLOSED") node->k = std::min(node->h, h_new);
    node->h = h_new;
    node->t = "OPEN";
    openlist.insert(std::make_pair(node->k, node));
}

double DStar::costFunc(DNode& node1, DNode& node2)
{
    if (isCollision(node1.position,node2.position))
    {
        return 10000.0;
    }
    int dx = abs(node1.position.first - node2.position.first);
    int dy = abs(node1.position.second - node2.position.second);

    return std::hypot(dx,dy);
}

bool DStar::isCollision(pair<int, int> node1, pair<int, int> node2) {
    // if (board_size <= node1.first < grid.size()-board_size && board_size <= node1.second < grid[0].size()-board_size &&
    //     board_size <= node2.first < grid.size()-board_size && board_size <= node2.second < grid[0].size()-board_size) 
    if (board_size <= node1.first && node1.first < grid.size()-board_size &&
        board_size <= node1.second && node1.second < grid[0].size()-board_size &&
        board_size <= node2.first && node2.first < grid.size()-board_size &&
        board_size <= node2.second && node2.second < grid[0].size()-board_size)
    {
        if (grid[node1.first][node1.second] == 0 && grid[node2.first][node2.second] == 0) 
        {
            int d1 = node1.first - node2.first;
            int d2 = node1.second - node2.second;
            if (d1 != 0 && d2 != 0)
            {
                if (grid[node1.first][node2.second]==0 && grid[node2.first][node1.second]==0){return false;}
            }
            else{return false;}
        }
    }
    return true;
}

void DStar::modify_cost(DNode* node, DNode* node_parent)
{
    if (node->t == "CLOSED")
    {
        insert(node,node_parent->h+costFunc(*node,*node_parent));
    }
    while (true)
    {
        double k_min = process_state();
        if (k_min>=node->h){
            break;
        }
    }
}

void DStar::delete_node(DNode* node) {
    node->t = "CLOSED";
    openlist.erase(openlist.begin());

}

double DStar::extractPath() {
    DNode* node = startNode;
    double cost = 0;
    path.push_back(node->position);
    while (node->position != goalNode->position) {
        DNode* parent = closed[node->parent];
        cost += costFunc(*node, *parent);
        node = parent;
        path.push_back(node->position);
    }

    return cost;
}

void DStar::onMouse(int event, int x, int y, int, void* userdata) {
    DStar* self = static_cast<DStar*>(userdata);
    if (event ==  EVENT_LBUTTONDOWN) {
        x = x/20;
        y = y/20;
        self->update(y, x);
    }
}

void DStar::update(int y, int x) {
    if (y < board_size || y > grid.size() - board_size - 1 ||
        x < board_size || x > grid[0].size() - board_size - 1) {
        std::cout << "Please choose right area!" << std::endl;
    } 
    else {
        // Clear the visited nodes.
        visited.clear();

        // Update obstacles.
        if (grid[y][x] == 0) {
            grid[y][x] = 1;
            std::cout<<"Add obstacle at ("<<y<<","<<x<<")"<<std::endl;
            path.clear();
            DNode* node = closed[startNode->position];
            double cost = 0;
            while (node->position != goalNode->position) {
                DNode* parent_node = closed[node->parent];
                if (isCollision(node->position, parent_node->position)) {
                    modify_cost(node, parent_node);
                    continue;
                }
                path.push_back(node->position);
                cost += costFunc(*node, *parent_node);
                node = parent_node;
            }
            path.push_back(goalNode->position);

            // Visualize again!
            visualize(cost);
            printf("Replan finished!\n");
        }
    }
}

void DStar::visualize(double cost) {
    printf("visualize\n");
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
     VideoWriter video("DStar_animation.mp4",  VideoWriter::fourcc('m', 'p', '4', 'v'), 100,  Size(img_width, img_height));
    
    // Create image for each frame
     Mat img(img_height, img_width, CV_8UC3,  Scalar(255, 255, 255));
    // Title and Cost
    string title = "DStar";
    putText(img, title,  Point(550, 40),  FONT_HERSHEY_SIMPLEX, 0.8,  Scalar(0, 0, 0), 2);
    string cost_text = "Cost: " + std::to_string(cost);
    putText(img, cost_text,  Point(500, 80),  FONT_HERSHEY_SIMPLEX, 0.8,  Scalar(0, 0, 0), 2);

    // Draw Obstacles
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            if (grid[i][j] == 1) {
                rectangle(img,  Point(j * cell_size, i * cell_size),
                           Point((j + 1) * cell_size, (i + 1) * cell_size),
                           Scalar(0, 0, 0),  FILLED);
            }
        }
    }

    // Draw start and goal point
    rectangle(img,  Point(startNode->position.second * cell_size, startNode->position.first * cell_size),
             Point((startNode->position.second + 1) * cell_size, (startNode->position.first + 1) * cell_size),
             Scalar(0, 255, 0),  FILLED);

    rectangle(img,  Point(goalNode->position.second * cell_size, goalNode->position.first * cell_size),
             Point((goalNode->position.second + 1) * cell_size, (goalNode->position.first + 1) * cell_size),
             Scalar(0, 255, 255),  FILLED);

    // Draw visited areas
    for (size_t i=1;i<visited.size();++i) 
    {
        const auto& node = visited[i];
        rectangle(img,  Point(node.second * cell_size, node.first * cell_size),
         Point((node.second + 1) * cell_size, (node.first + 1) * cell_size),
         Scalar(128, 128, 128),  FILLED);
        
        video.write(img);
    }

    // Draw path
    for (size_t i = 1; i < path.size(); ++i) 
    {
         Point p1(path[i - 1].second * cell_size + cell_size / 2,
                path[i - 1].first * cell_size + cell_size / 2);
         Point p2(path[i].second * cell_size + cell_size / 2,
                path[i].first * cell_size + cell_size / 2);
         line(img, p1, p2,  Scalar(0, 0, 255), 2);

        video.write(img);
    }

    video.release();
    imwrite("DStar_result.png", img);
    imshow("DStar", img);
    printf("Plot finished!\n");
    // waitKey(0);
    // destroyAllWindows();
}