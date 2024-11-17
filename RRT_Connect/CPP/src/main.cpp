#include "rrt_connect.hpp"


int main() {
    int width = 60;
    int height = 40;
    int board_size = 1;

    vector<Rect> boundary = {
        Rect(board_size, 3*board_size, board_size, height-4*board_size),
        Rect(board_size, 3*board_size, width-2*board_size, board_size),
        Rect(board_size, height-2*board_size, width-2*board_size, board_size),
        Rect(width-2*board_size, 3*board_size, board_size, height-4*board_size)
    };

    vector<Rect> obs_rectangle = {
        Rect(8, 25, 5, 10),
        Rect(35, 10, 5, 5),
        Rect(35, 25, 5, 8)
    };

    vector<Point3f> obs_circle = {
        Point3f(45, 26, 3),
        Point3f(32, 17, 2),
        Point3f(23, 18, 2),
        Point3f(24, 32, 3),
        Point3f(30, 25, 3)
    };

    RRTConnect rrt_connect(pair<double,double>(12, 12), pair<double,double>(45, 35), width, height, board_size, 5000, 1.0, 0.05, boundary, obs_rectangle, obs_circle);
    rrt_connect.run();

    return 0;
}
