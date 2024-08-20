from map import Map
from jps import JPS

# import sys
# # 设置递归深度限制为 1000000
# sys.setrecursionlimit(1000000)

if __name__ == '__main__':
    # Initialize the class Map and generate a map with obstacles
    map = Map()
    width, height = 60, 40
    board_size = 5
    obstacle = []
    for row in range(5,20):
        obstacle.append((row,25))
    for col in range(15,26):
        obstacle.append((20,col))
    for row in range(20,35):
        obstacle.append((row,35))
    for row in range(5,21):
        obstacle.append((row,45))
    
    grid = map.grid_map_user_defined(width,height,board_size,obstacle)
    # map.save_map()

    # Set the start and goal point
    start = (10,10)
    goal = (30,50)

    # Initialize the planner
    planner = JPS(grid,start,goal,board_size)
    # Do plan
    cost = planner.plan()
    if cost == -1:
        print("Fail to find the proper path\n")
    else:
        print("Find the best path successfully\n")
    # Visualize the result
    planner.visualize_grid(cost)
