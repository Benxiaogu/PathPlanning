"""
    Filename: jps.py
    Description: Plan path using Jump-Point-Search Algorithm
    Author: Benxiaogu:https://github.com/Benxiaogu
    Date: 2024-08-18
"""
import heapq
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class JPS:
    def __init__(self,grid,start,goal,board_size):
        self.grid = grid
        self.start = self.Node(start,0,0)
        self.goal = self.Node(goal,0,0)
        self.board_size = board_size
        self.path = []

    class Node:
        def __init__(self, position, g, h, parent=None):
            self.position = position    # position of node
            self.g = g                  # distance from node to start
            self.h = h                  # heuristic value from node to goal
            self.parent = parent        # parent node

        def __eq__(self, other) -> bool:
            return self.position == other.position
        
        def __lt__(self, other):
            # Prioritise nodes based on heuristic value
            return self.g+self.h < other.g+other.h or (self.g+self.h==other.g+other.h and self.h<other.h)

    def plan(self):
        open = []
        closed = []
        self.searched = [] # Used to record nodes that are searched

        nexts = [(-1,0),(0,1),(0,-1),(1,0),(-1,1),(1,1),(1,-1),(-1,-1)]

        self.start.h = self.heuristic(self.start.position)
        heapq.heappush(open, self.start)

        while open:
            # Select the node closest to the start node
            current_node = heapq.heappop(open)
            if current_node.position in closed:
                continue

            # self.searched.append(current_node.position)
            closed.append(current_node.position)
            
            # Find the goal
            if current_node == self.goal:
                self.goal = current_node
                while current_node:
                    self.path.append(current_node.position)
                    current_node = current_node.parent
                self.path = self.path[::-1]

                return self.goal.g

            for next in nexts:
                jumppoint = self.Jumping(current_node.position,next)
                if jumppoint and jumppoint not in closed:
                    h = self.heuristic(jumppoint)
                    g = current_node.g+self.calculate_cost(current_node.position, jumppoint)
                    jp_node = self.Node(jumppoint,g,h,current_node)
                    heapq.heappush(open,jp_node)
                    self.searched.append(jumppoint)
                    if jp_node == self.goal:
                        break

        
        return -1
    
    def Jumping(self, node, direction):
        """
            Return jump point or None if fail to search
        """
        new_node = (node[0]+direction[0], node[1]+direction[1])
        
        if self.grid[new_node[0]][new_node[1]] == 1:
            return None

        if new_node == self.goal.position:
            return new_node

        # Find forced neighbor at horizaontal and vertical direction
        if self.findForcedNeighbor(new_node, direction):
            return new_node
        
        # If current direction of search is diagonal, then search jump point horizaontally or vertically
        if direction[0]!=0 and direction[1]!=0:
            y_dir = (direction[0],0)
            x_dir = (0,direction[1])
            if self.Jumping(new_node,x_dir) or self.Jumping(new_node,y_dir):
                return new_node

        return self.Jumping(new_node,direction)

    def findForcedNeighbor(self, node, direction):
        """
            If forced neighbor exists, return Ture otherwise False
        """
        y,x = node

        # vertical
        if direction[0]!=0 and direction[1]==0:
            if (self.grid[y][x+1] != 0 and self.grid[y+direction[0]][x+1] == 0) or \
                (self.grid[y][x-1] != 0 and self.grid[y+direction[0]][x-1] == 0):
                return True
        
        # horizontal
        if direction[0]==0 and direction[1]!=0:
            if (self.grid[y+1][x] != 0 and self.grid[y+1][x+direction[1]] == 0) or \
                (self.grid[y-1][x] != 0 and self.grid[y-1][x+direction[1]] == 0):
                return True
        
        # diagonal
        if direction[0]!=0 and direction[1]!=0:
            if (self.grid[y-direction[0]][x] != 0 and self.grid[y-direction[0]][x+direction[1]] == 0) or \
                (self.grid[y][x-direction[1]] != 0 and self.grid[y+direction[0]][x-direction[1]] == 0):
                return True
        
        return False

    # def heuristic(self, node):
    #     # Manhattan distance from current node to goal node
    #     return abs(node[0] - self.goal.position[0]) + abs(node[1] - self.goal.position[1])
    
    # def heuristic(self, node):
    #     # Chebyshev Distance
    #     D = 1
    #     D2 = math.sqrt(2)
    #     dx = abs(node[0] - self.goal.position[0])
    #     dy = abs(node[1] - self.goal.position[1])
    #     return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    
    def heuristic(self, node):
        # Euclidean Distance
        D = 1
        dy = abs(node[0] - self.goal.position[0])
        dx = abs(node[1] - self.goal.position[1])
        return D * math.sqrt(dx * dx + dy * dy)
    
    def calculate_cost(self, start, end):
        # The distance from the current node to the next jump point
        return math.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)

    def visualize_grid(self, cost):
        fig, ax = plt.subplots()
        grid = np.array(self.grid)

        plt.imshow(grid, cmap='Greys', origin='upper')

        plt.title("JPS\n"+"Cost: "+str(cost))

        # Mark the start and goal point
        plt.scatter(self.start.position[1], self.start.position[0], c='green', marker='o', s=60)
        plt.scatter(self.goal.position[1], self.goal.position[0], c='blue', marker='o', s=60)

        # # Mark locations which has been visited
        # for node in self.searched:
        #     plt.gca().add_patch(plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5))
        
        # # Mark path
        # if self.path:
        #     path_x, path_y = zip(*self.path)
        #     plt.plot(path_y, path_x, c='red', linewidth=2)
        
        visited_patches = []
        path_line, = ax.plot([], [], c='red', linewidth=2)
        # for order in range(len(self.searched)):
        #     node = self.searched[order]
        #     plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5)

        def update(frame):
            if frame < len(self.searched):
                node = self.searched[frame]
                patch = plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5)
                visited_patches.append(patch)
                ax.add_patch(patch)
            elif self.path:
                path_x, path_y = zip(*self.path)
                path_line.set_data(path_y, path_x)

            return visited_patches + [path_line]*20
    
        ani = animation.FuncAnimation(fig, update, frames=len(self.searched) + 20, interval=50, repeat=False)
        ani.save("map_animate.gif",writer='pillow')
        # ani.save("map_animate.mp4",writer='ffmpeg')

        plt.savefig("map_animate.png",bbox_inches='tight')
        plt.show()
        