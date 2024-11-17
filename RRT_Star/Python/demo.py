import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from RRT.Python.map import Map
from rrt_star import RRTStar
# from test import RRTStar

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
    search_r = 10.0
    rrt_star = RRTStar(start,goal,obstacles,search_r,board_size,max_try,max_dist,goal_sample_rate,map)

    rrt_star.run()