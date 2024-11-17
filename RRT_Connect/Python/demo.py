from map import Map
from rrt_connect import RRTConnect

if __name__ == '__main__':
    map = Map(60,40)
    start = (12,12)
    goal = (45,35)
    obstacles = None
    max_try = 10000
    max_dist = 0.5
    goal_sample_rate = 0.05
    area = None
    board_size = 5
    rrt_connect = RRTConnect(start,goal,obstacles,board_size,max_try,max_dist,goal_sample_rate,map)

    rrt_connect.run()