from map import Map
from dijkstra import Dijkstra

if __name__ == '__main__':
    # Initialize the class Map and generate a map with obstacles
    map = Map()
    # grid = map.grid_map_random_generator(40,40,0.2)
    width, height = 60, 40
    board_size = 5
    obstacle = []
    # obstacle = [(4,5),(3,5),(2,5),(5,5),(6,5)]
    # obstacle = [(4,5),(4,4),(4,3),(5,5),(6,5)]
    for row in range(5,20):
        obstacle.append((row,25))
    for col in range(15,26):
        obstacle.append((20,col))
    for row in range(20,35):
        obstacle.append((row,35))
    for row in range(5,20):
        obstacle.append((row,45))
    
    grid = map.grid_map_user_defined(width,height,board_size,obstacle)

    # Set the start and goal point
    start = (10,10)
    goal = (30,50)
    # Set the secure scale of robot
    radius = 1

    # Initialize the planner
    planner = Dijkstra(grid,start,goal,radius,board_size)
    # Do plan
    cost = planner.plan()
    # Visualize the result
    planner.visualize_grid(cost)
