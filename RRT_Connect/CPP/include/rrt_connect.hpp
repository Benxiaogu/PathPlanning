#ifndef RRTConnect_HPP
#define RRTConnect_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class RRTConnect {
public:
    struct Node {
        pair<double,double> position;
        pair<double,double> parent;
        double cost;
        bool st;

        // 默认构造函数
        Node() : position({0, 0}), parent({0,0}), cost(0.0), st(false) {}

        Node(pair<double,double> pos, pair<double,double> par={0,0}, double c = 0.0, bool t=true) 
            : position(pos), parent(par), cost(c), st(t) {}
    };

    RRTConnect(pair<double,double> start, pair<double,double> goal,int width, int height, int board_size, int max_try, double max_dist, double goal_sample_rate, vector<Rect> boundary,vector<Rect> obs_rectangle, vector<Point3f> obs_circle);

    void run();

private:
    Node start, goal;
    int width,height;
    int board_size, max_try;
    double max_dist, goal_sample_rate;
    vector<Node> searched;
    vector<Rect> boundary, obs_rectangle;
    vector<Point3f> obs_circle;
    float inflation;
    map<pair<double,double>,Node> nodes_forward, nodes_backward;

    pair<vector<pair<double,double>>,vector<RRTConnect::Node>> plan(double& cost);
    Node get_random_node();
    Node get_nearest_neighbor(map<pair<double,double>, Node>& node_list, Node node);
    Node get_new_node(Node node_rand, Node node_near);
    bool isCollision(Node* node1, Node* node2);
    bool isInBoundary(Node* node);
    bool isInObstacles(Node* node);
    float cross(const pair<double,double>& p1, const pair<double,double>& p2, const pair<double,double>& p3);
    bool isInterRect(Node* node1, Node* node2, const cv::Rect& rect);
    bool isInterCircle(Node* node1, Node* node2, Point3f circle);
    double distance(Node node1, Node node2);
    pair<double,vector<pair<double,double>>> extractPath(Node node_middle, map<pair<double,double>,Node> nodes_f, map<pair<double,double>,Node> nodes_b);
    vector<RRTConnect::Node> get_expand(map<pair<double,double>,Node>& nodes_f, map<pair<double,double>,Node>& nodes_b);
    void visualize(double cost, vector<pair<double,double>>& path);
};
#endif // RRTConnect_HPP