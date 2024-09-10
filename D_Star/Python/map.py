import random
import matplotlib.pyplot as plt
import numpy as np

class Map:
    def __init__(self):
        self.grid = []

    def grid_map_user_defined(self, width, height, board_size, obstacles):
        self.grid = [[0 for _ in range(width)] for _ in range(height)]
        self.obstacles = set(obstacles)
        # boundary of environment
        for i in range(board_size, width-board_size+1):
            self.obstacles.add((board_size, i))
            self.obstacles.add((height-board_size,i))
        for k in range(board_size, height-board_size+1):
            self.obstacles.add((k,board_size))
            self.obstacles.add((k,width-board_size))
        
        # set obstacles in map
        for row,col in self.obstacles:
            self.grid[row][col] = 1

        return self.grid

    def save_map(self):
        if self.grid:
            map = np.array(self.grid)
            plt.imshow(map, cmap='Greys', origin='lower')
            plt.savefig("gridmap.png")
            # plt.show()
        else:
            print("Please generate grid map firstly.")