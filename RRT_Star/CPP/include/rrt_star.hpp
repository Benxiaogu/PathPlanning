#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <algorithm>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class RRTStar {
public:
    struct Node {
        pair<double,double> position;
        pair<double,double> parent;
        double cost;
        bool st;

        Node() : position({0, 0}), parent({0,0}), cost(0.0), st(false) {}

        Node(pair<double,double> pos, pair<double,double> par={0,0}, double c = 0.0, bool t=true) 
            : position(pos), parent(par), cost(c), st(t) {}
    };

    RRTStar(pair<double,double> start, pair<double,double> goal,int width, int height, float search_r, int board_size, int max_try, double max_dist, double goal_sample_rate, vector<Rect> boundary,vector<Rect> obs_rectangle, vector<Point3f> obs_circle);

    void run();

private:
    Node start, goal;
    int width,height;
    float search_r;
    int board_size, max_try;
    double max_dist, goal_sample_rate;
    vector<Node> searched;
    vector<Rect> boundary, obs_rectangle;
    vector<Point3f> obs_circle;
    float inflation;
    map<pair<double,double>, Node, less<>> closed_list;

    vector<pair<double,double>> plan(double& cost);
    Node get_random_node();
    Node get_nearest_neighbor(map<pair<double,double>, Node, less<>>& node_list, Node node);
    Node get_new_node(Node node_rand, Node node_near);
    bool isCollision(Node* node1, Node* node2);
    bool isInBoundary(Node* node);
    bool isInObstacles(Node* node);
    float cross(const pair<double,double>& p1, const pair<double,double>& p2, const pair<double,double>& p3);
    bool isInterRect(Node* node1, Node* node2, const cv::Rect& rect);
    bool isInterCircle(Node* node1, Node* node2, Point3f circle);
    double distance(Node node1, Node node2);
    vector<pair<double,double>> extractPath(map<pair<double,double>, Node, less<>>& closed_list);
    void visualize(double cost, vector<pair<double,double>>& path);
};
#endif // RRT_STAR_HPP