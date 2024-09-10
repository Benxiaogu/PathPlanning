#ifndef DSTAR_HPP
#define DSTA_HPP

#include <iostream>
#include <vector>
#include <set>
#include <map>

using namespace std;

struct DNode {
    pair<int, int> position;
    pair<int, int> parent;
    string t;  // DNode status: "NEW", "OPEN", "CLOSED"
    double h;  // Cost from goal to current node
    double k;  // Minimum cost from goal to current node in history

    // DNode(pair<int, int> position, pair<int, int> parent, string t, double h, double k) 
    //     : position(position), parent(parent), t(t), h(h), k(k) {}

    bool operator==(const DNode& other) const {
        return position == other.position;
    }

    bool operator<(const DNode& other) const {
        return k + h < other.k + other.h || (k + h == other.k + other.h && h < other.h);
    }
};

struct Neighbor 
{
    pair<int, int> direction;
    double cost;
    Neighbor(pair<int, int> dir, double c) : direction(dir), cost(c) {}
};

class DStar
{
public:
    DStar(vector<vector<int>> grid, pair<int, int> start, pair<int, int> goal, int radius, int board_size);
    tuple<vector<DNode*>, vector<double>> get_neighbors(DNode* dnode);
    void run();
    double plan();
    double process_state();
    DNode* min_state();
    double get_kmin();
    void delete_node(DNode* node);
    void modify_cost(DNode* node, DNode* node_parent);
    void insert(DNode* node, double h_new);
    double costFunc(DNode& node1, DNode& node2);
    bool isCollision(pair<int,int> node1, pair<int,int> node2);
    double extractPath();
    void update(int y,int x);

    void visualize(double cost);
    static void onMouse(int event, int x, int y, int, void* userdata);

private:
    vector<vector<int>> grid; 
    DNode* startNode;
    DNode* goalNode;
    int radius;
    int board_size;
    // vector<DNode*> openlist;
    std::multimap<double, DNode*> openlist;  // open list, ascending order
    vector<pair<int,int>> path;
    vector<pair<int,int>> visited;
    map<pair<int, int>, DNode*> closed;

};
#endif // DSTAR_HPP