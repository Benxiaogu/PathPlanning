#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rrt.hpp"

using namespace std;
using namespace cv;

// RRT Constructor
RRT::RRT(pair<double,double> start, pair<double,double> goal, int width, int height, int board_size, int max_try, double max_dist, double goal_sample_rate, vector<Rect> boundary,vector<Rect> obs_rectangle, vector<Point3f> obs_circle)
    : start(start, start, 0), goal(goal, goal, 0), width(width), height(height), board_size(board_size), max_try(max_try),
      max_dist(max_dist), goal_sample_rate(goal_sample_rate), boundary(boundary), obs_rectangle(obs_rectangle), obs_circle(obs_circle) 
{
    inflation = 0.1;
}

void RRT::run() 
{
    double cost;
    vector<pair<double,double>> path = plan(cost);
    printf("Plan finished!\n");
    visualize(cost, path);
}

vector<pair<double,double>> RRT::plan(double& cost) 
{
    map<pair<double,double>, Node, less<>> closed_list;
    searched.push_back(start);
    closed_list[start.position] = start;

    for (int i = 0; i < max_try; ++i) {
        Node node_rand = get_random_node();
        // cout<<"node_rand: "<<node_rand.position.first<<","<<node_rand.position.second<<endl;

        if (closed_list.find(node_rand.position) != closed_list.end()) {
            continue;
        }

        Node node_near = get_nearest_neighbor(closed_list, node_rand);
        // cout<<"node_near: "<<node_near.position.first<<","<<node_near.position.second<<endl;
        Node node_new = get_new_node(node_rand, node_near);
        if (node_new.st) {
            // cout<<"node_new: "<<node_new.position.first<<","<<node_new.position.second<<endl;
            closed_list[node_new.position] = node_new;
            searched.push_back(node_new);
            double dist = distance(node_new, goal);
            if (dist <= max_dist && !isCollision(&node_new, &goal)) {
                goal.parent = node_new.position;
                goal.cost = node_new.cost + distance(goal, node_new);
                closed_list[goal.position] = goal;
                cost = goal.cost;
                // printf("Extract path.\n");
                return extractPath(closed_list);
            }
        }
        // delete node_rand;
    }
    cost = 0;
    printf("Path is null!\n");
    vector<pair<double,double>> path;
    path.push_back(start.position);
    return path;
}

RRT::Node RRT::get_random_node() {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis_x(0, width), dis_y(0,height);
    if (dis_x(gen) > goal_sample_rate) {
        return Node(pair<double,double>(dis_x(gen), dis_y(gen)));
    } else {
        return Node(goal.position);
    }
}

RRT::Node RRT::get_nearest_neighbor(map<pair<double,double>, Node, less<>>& node_list, Node node) {
    double min_dist = numeric_limits<double>::max();
    Node nearest_node(node);
    for (auto& n : node_list) {
        // cout<<"n: "<<n.second->position.first<<","<<n.second->position.second<<endl;
        double dist = distance(n.second, node);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_node = n.second;
        }
    }
    return nearest_node;
}

RRT::Node RRT::get_new_node(Node node_rand, Node node_near) {
    double dx = node_rand.position.first - node_near.position.first;
    double dy = node_rand.position.second - node_near.position.second;
    double dist = hypot(dx, dy);
    double theta = atan2(dy, dx);

    double d = min(max_dist, dist);
    pair<double,double> new_pos(node_near.position.first + d * cos(theta), node_near.position.second + d * sin(theta));

    Node node_new(new_pos, node_near.position, node_near.cost + d);
    if (isCollision(&node_new, &node_near)) {
        node_new.st = false;
        // std::cout<<"node_new.st: "<<node_new.st<<std::endl;
    }
    else{node_new.st = true;}

    return node_new;
}

bool RRT::isCollision(Node* node1, Node* node2) {
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

bool RRT::isInBoundary(Node* node)
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

bool RRT::isInObstacles(Node* node) {
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

float RRT::cross(const pair<double,double>& p1, const pair<double,double>& p2, const pair<double,double>& p3) {
    float x1 = p2.first - p1.first;
    float y1 = p2.second - p1.second;
    float x2 = p3.first - p1.first;
    float y2 = p3.second - p1.second;
    return x1 * y2 - x2 * y1;
}

bool RRT::isInterRect(Node* node1, Node* node2, const cv::Rect& rect) {
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

bool RRT::isInterCircle(Node* node1, Node* node2, Point3f circle) {
    // Calculate intersection with circle (similar to the Python implementation)
    double dx = node2->position.first - node1->position.first;
    double dy = node2->position.second - node1->position.second;

    double t = ((circle.x - node1->position.first) * dx + (circle.y - node1->position.second) * dy) / (dx * dx + dy * dy);

    if (0 <= t && t <= 1) {
        double closest_x = node1->position.first + t * dx;
        double closest_y = node1->position.second + t * dy;
        double distance = hypot(circle.x - closest_x, circle.y - closest_y);

        return distance <= circle.z+inflation;
    }

    return false;
}

double RRT::distance(Node node1, Node node2) 
{
    return hypot(node2.position.first - node1.position.first, node2.position.second - node1.position.second);
}

vector<pair<double,double>> RRT::extractPath(map<pair<double,double>, Node, less<>>& closed_list) 
{
    vector<pair<double,double>> path;
    Node node(goal);
    while (node.position != start.position) {
        path.push_back(node.position);
        // Node node_parent = *closed_list[node.parent->position];
        pair<double,double> parent = node.parent;
        node = closed_list[parent];
        // node = *node.parent;
        // cout<<"node: "<<node.position.first<<","<<node.position.second<<endl;
    }
    path.push_back(start.position);
    reverse(path.begin(), path.end());
    return path;
}

void RRT::visualize(double cost, vector<pair<double,double>>& path) {
    int cell_size = 20;

    VideoWriter video("RRT_animation.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 100, Size(width*cell_size, height*cell_size));

    Mat img = Mat::zeros(height*cell_size, width*cell_size, CV_8UC3);
    img.setTo(Scalar(255, 255, 255));

    // Title and Cost
    std::string title = "RRT";
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
    for (size_t i=1;i<searched.size()-1;++i) 
    {
        Node node = searched[i];
        line(img, Point2f(node.position.first*cell_size,node.position.second*cell_size), Point2f(node.parent.first*cell_size,node.parent.second*cell_size), Scalar(0, 100, 0), 2);
        
        video.write(img);
    }

    // Draw the path
    for (size_t i = 0; i < path.size() - 1; ++i) {
        line(img, Point2f(path[i].first*cell_size,path[i].second*cell_size), Point2f(path[i + 1].first*cell_size,path[i+1].second*cell_size), Scalar(0, 0, 255), 2);
    }

    video.release();
    // Show the image
    imwrite("RRT.png", img);
    imshow("RRT Path", img);
    waitKey(0);
    destroyAllWindows();
    }