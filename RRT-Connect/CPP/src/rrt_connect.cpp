/**
 * @File: rrt_connect.cpp
 * @Brief: Description
 *
 * @Author: Benxiaogu
 * @Github: https://github.com/Benxiaogu
 * @CSDN: https://blog.csdn.net/weixin_51995147?type=blog
 *
 * @Date: 2024-11-13
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rrt_connect.hpp"

using namespace std;
using namespace cv;

// RRTConnect Constructor
RRTConnect::RRTConnect(pair<double,double> start, pair<double,double> goal, int width, int height, int board_size, int max_try, double max_dist, double goal_sample_rate, vector<Rect> boundary,vector<Rect> obs_rectangle, vector<Point3f> obs_circle)
    : start(start, start, 0), goal(goal, goal, 0), width(width), height(height), board_size(board_size), max_try(max_try),
      max_dist(max_dist), goal_sample_rate(goal_sample_rate), boundary(boundary), obs_rectangle(obs_rectangle), obs_circle(obs_circle) 
{
    inflation = 0.1;
}

void RRTConnect::run() 
{
    double cost=0;
    vector<pair<double,double>> path;
    auto result = plan(cost);
    path = result.first;
    searched = result.second;
    printf("Plan finished!\n");
    visualize(cost, path);
}

pair<vector<pair<double,double>>,vector<RRTConnect::Node>> RRTConnect::plan(double& cost) 
{
    // vector<pair<double,double>> path;
    map<pair<double,double>, Node> nodes_f = {{start.position, start}};
    map<pair<double,double>, Node> nodes_b = {{goal.position, goal}};

    for (int i = 0; i < max_try; ++i) {
        Node node_rand = get_random_node();
        Node node_near = get_nearest_neighbor(nodes_f, node_rand);
        Node node_new = get_new_node(node_rand, node_near);
        if (node_new.st) {
            nodes_f[node_new.position] = node_new;

            Node node_near_b = get_nearest_neighbor(nodes_b, node_new);
            Node node_new_b = get_new_node(node_new, node_near_b);

            if (node_new_b.st){
                nodes_b[node_new_b.position] = node_new_b;

                while (true)
                {
                    if (nodes_b.find(node_new.position) != nodes_b.end()){
                        auto [cost_path, path] = extractPath(node_new, nodes_f, nodes_b);
                        cost = cost_path;
                        vector<Node> expand = get_expand(nodes_f, nodes_b);
                        printf("Be in iteration %d.\n",i);

                        return {path, expand};
                    }

                    Node node_new_b2 = get_new_node(node_new, node_new_b);
                    if (node_new_b2.st){
                        nodes_b[node_new_b2.position] = node_new_b2;
                        node_new_b = node_new_b2;
                    }
                    else{
                        break;
                    }
                }
            }
        }

        if (nodes_b.size() < nodes_f.size()){
            swap(nodes_f,nodes_b);
        }
    }

    return {{},{}};
}

RRTConnect::Node RRTConnect::get_random_node() {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis_x(0, width), dis_y(0,height);
    if (dis_x(gen) > goal_sample_rate) {
        return Node(pair<double,double>(dis_x(gen), dis_y(gen)));
    } else {
        return Node(goal.position);
    }
}

RRTConnect::Node RRTConnect::get_nearest_neighbor(map<pair<double,double>, Node>& node_list, Node node) {
    double min_dist = numeric_limits<double>::max();
    Node nearest_node(node);
    for (auto& n : node_list) 
    {
        double dist = distance(n.second, node);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_node = n.second;
        }
    }
    return nearest_node;
}

RRTConnect::Node RRTConnect::get_new_node(Node node_rand, Node node_near) {
    double dx = node_rand.position.first - node_near.position.first;
    double dy = node_rand.position.second - node_near.position.second;
    double dist = hypot(dx, dy);
    double theta = atan2(dy, dx);

    double d = min(max_dist, dist);
    pair<double,double> new_pos(node_near.position.first + d * cos(theta), node_near.position.second + d * sin(theta));

    Node node_new(new_pos, node_near.position, node_near.cost + d);
    if (isCollision(&node_new, &node_near)) {
        node_new.st = false;
    }
    else{node_new.st = true;}

    return node_new;
}

bool RRTConnect::isCollision(Node* node1, Node* node2) {
    if (!isInBoundary(node1)){
        return true;
    }
    if (isInObstacles(node1) || isInObstacles(node2)) {
        return true;
    }

    for (auto& rect : obs_rectangle) {
        if (isInterRect(node1, node2, rect)) {
            return true;
        }
    }

    for (auto& circle : obs_circle) {
        if (isInterCircle(node1, node2, circle)) {
            return true;
        }
    }

    return false;
}

bool RRTConnect::isInBoundary(Node* node)
{
    pair<double,double> node_position = node->position;
    int x1 = boundary[0].x+boundary[0].width;
    int y1 = boundary[1].y+boundary[1].height;
    int x2 = boundary[3].x;
    int y2 = boundary[2].y;
    // printf("x1: %d\n",x1);
    // printf("y1: %d\n",y1);
    // printf("x2: %d\n",x2);
    // printf("y2: %d\n",y2);

    if (node_position.first > x1 && node_position.first < x2 && node_position.second > y1 && node_position.second < y2)
    {
        return true;
    }
    return false;
}

bool RRTConnect::isInObstacles(Node* node) {
    pair<double,double> pos = node->position;
    Point2f point(pos.first, pos.second);

    for (auto& rect : obs_rectangle) {
        Rect rec = rect;
        rec.x -= inflation;
        rec.y -= inflation;
        rec.width += 2*inflation;
        rec.height += 2*inflation;
        if (rec.contains(point)) {
            return true;
        }
    }

    for (auto& circle : obs_circle) {
        double dist = hypot(pos.first - circle.x, pos.second - circle.y);
        if (dist <= circle.z+inflation) {
            return true;
        }
    }

    return false;
}

float RRTConnect::cross(const pair<double,double>& p1, const pair<double,double>& p2, const pair<double,double>& p3) {
    float x1 = p2.first - p1.first;
    float y1 = p2.second - p1.second;
    float x2 = p3.first - p1.first;
    float y2 = p3.second - p1.second;
    return x1 * y2 - x2 * y1;
}

bool RRTConnect::isInterRect(Node* node1, Node* node2, const cv::Rect& rect) {
    // Calculate the four vertices of the rectangle considering the inflation
    std::vector<pair<double,double>> vertex = {
        pair<double,double>(rect.x - inflation, rect.y - inflation),
        pair<double,double>(rect.x + rect.width + inflation, rect.y - inflation),
        pair<double,double>(rect.x + rect.width + inflation, rect.y + rect.height + inflation),
        pair<double,double>(rect.x - inflation, rect.y + rect.height + inflation)
    };

    pair<double,double> p1 = node1->position;
    pair<double,double> p2 = node2->position;

    // Check if any rectangle edge intersects the line segment from node1 to node2
    for (size_t i = 0; i < vertex.size(); ++i) {
        pair<double,double> v1 = vertex[i];
        pair<double,double> v2 = vertex[(i + 1) % vertex.size()];  // Ensuring cyclic order

        if (std::max(p1.first, p2.first) >= std::min(v1.first, v2.first) && std::min(p1.first, p2.first) <= std::max(v1.first, v2.first) &&
            std::max(p1.second, p2.second) >= std::min(v1.second, v2.second) && std::min(p1.second, p2.second) <= std::max(v1.second, v2.second)) {

            if (cross(v1, v2, p1) * cross(v1, v2, p2) <= 0 &&
                cross(p1, p2, v1) * cross(p1, p2, v2) <= 0) {
                return true;
            }
        }
    }

    return false;
}

bool RRTConnect::isInterCircle(Node* node1, Node* node2, Point3f circle) {
    // Calculate intersection with circle (similar to the Python implementation)
    double dx = node2->position.first - node1->position.first;
    double dy = node2->position.second - node1->position.second;

    if (dx * dx + dy * dy == 0){
        return false;
    }

    double t = ((circle.x - node1->position.first) * dx + (circle.y - node1->position.second) * dy) / (dx * dx + dy * dy);

    if (0 <= t && t <= 1) {
        double closest_x = node1->position.first + t * dx;
        double closest_y = node1->position.second + t * dy;
        double distance = hypot(circle.x - closest_x, circle.y - closest_y);

        return distance <= circle.z+inflation;
    }

    return false;
}

double RRTConnect::distance(Node node1, Node node2) 
{
    return hypot(node2.position.first - node1.position.first, node2.position.second - node1.position.second);
}

pair<double,vector<pair<double,double>>> RRTConnect::extractPath(Node node_middle, 
                                                    map<pair<double,double>, Node> nodes_f, 
                                                    map<pair<double,double>, Node> nodes_b) 
{
    if (nodes_b.find(start.position) != nodes_b.end()) {
        std::swap(nodes_f, nodes_b);
    }

    Node node = nodes_f[node_middle.position];

    vector<pair<double,double>> path_f = {node.position};
    double cost = node.cost;
    while (node.position != start.position) 
    {
        node = nodes_f[node.parent];
        path_f.push_back(node.position);
    }
    reverse(path_f.begin(), path_f.end());

    node = nodes_b[node_middle.position];
    vector<pair<double,double>> path_b;
    cost += node.cost;
    while (node.position != goal.position)
    {
        node = nodes_b[node.parent];
        path_b.push_back(node.position);
    }

    path_f.insert(path_f.end(),path_b.begin(),path_b.end());

    return {cost, path_f};
}

vector<RRTConnect::Node> RRTConnect::get_expand(map<pair<double,double>,Node>& nodes_f, map<pair<double,double>,Node>& nodes_b)
{
    vector<Node> expand;
    size_t tree_size = max(nodes_f.size(),nodes_b.size());

    auto it_forward = nodes_f.begin();
    auto it_back = nodes_b.end();
    for (size_t i=0; i<tree_size; i++)
    {
        if (it_forward != nodes_f.end())
        {
            expand.push_back(it_forward->second);
            ++it_forward;
        }
        if (it_back != nodes_b.begin())
        {
            expand.push_back(it_back->second);
            --it_back;
        }
    }

    return expand;
}

void RRTConnect::visualize(double cost, vector<pair<double,double>>& path) {
    int cell_size = 20;

    VideoWriter video("RRTConnect_animation.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(width*cell_size, height*cell_size));

    Mat img = Mat::zeros(height*cell_size, width*cell_size, CV_8UC3);
    img.setTo(Scalar(255, 255, 255));

    // Title and Cost
    std::string title = "RRTConnect";
    putText(img, title, Point(550, 20), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);
    std::string cost_text = "Cost: " + std::to_string(cost);
    putText(img, cost_text, Point(500, 50), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);

    // Draw boundary
    for (auto& rect : boundary) {
        rect.x *= cell_size;
        rect.y *= cell_size;
        rect.height *= cell_size;
        rect.width *= cell_size;

        rectangle(img, rect, Scalar(0, 0, 0), FILLED);
    }
    // Draw obstacles
    for (auto& rect : obs_rectangle) {
        rect.x *= cell_size;
        rect.y *= cell_size;
        rect.height *= cell_size;
        rect.width *= cell_size;

        rectangle(img, rect, Scalar(0, 0, 0), FILLED);
    }
    for (auto& circ : obs_circle) {
        circle(img, Point(circ.x*cell_size, circ.y*cell_size), circ.z*cell_size, Scalar(0, 0, 0), FILLED);
    }

    // Draw start and goal
    circle(img, Point2f(start.position.first*cell_size,start.position.second*cell_size), 5, Scalar(0, 255, 0), FILLED);
    circle(img, Point2f(goal.position.first*cell_size,goal.position.second*cell_size), 5, Scalar(255, 0, 0), FILLED);

    // Draw visited areas
    for (size_t i=1;i<searched.size();++i) 
    {
        Node node = searched[i];
        line(img, Point2f(node.position.first*cell_size,node.position.second*cell_size), Point2f(node.parent.first*cell_size,node.parent.second*cell_size), Scalar(0, 100, 0), 2);
        
        video.write(img);
    }

    // Draw the path
    for (size_t i = 0; i < path.size()-1; ++i) {
        line(img, Point2f(path[i].first*cell_size,path[i].second*cell_size), Point2f(path[i + 1].first*cell_size,path[i+1].second*cell_size), Scalar(0, 0, 255), 2);
        video.write(img);
    }

    video.release();
    // Show the image
    imwrite("RRTConnect.png", img);
    imshow("RRTConnect Path", img);
    waitKey(0);
    destroyAllWindows();
    }