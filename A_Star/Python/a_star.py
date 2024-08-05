"""
    Filename: a_star.py
    Description: Plan path using A* Algorithm
    Author: Benxiaogu:https://github.com/Benxiaogu
    Date: 2024-08-05
"""
import heapq
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class AStar:
    def __init__(self,grid,start,goal,board_size):
        self.grid = grid
        self.start = self.Node(start,0,0)
        self.goal = self.Node(goal,0,0)
        self.board_size = board_size

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
        
    class Neighbor:
        def __init__(self,directions,cost):
            self.directions = directions
            self.cost = cost
    
    def get_neighbors(self, node):
        neighbors = []
        distances = []
        nexts = [self.Neighbor((-1, 0),1), self.Neighbor((0, 1),1), self.Neighbor((0, -1),1), self.Neighbor((1,0),1),
                self.Neighbor((-1,1),math.sqrt(2)), self.Neighbor((1,1),math.sqrt(2)),self.Neighbor((1, -1),math.sqrt(2)), self.Neighbor((-1,-1),math.sqrt(2))]
        for next in nexts:
            neighbor = (node[0] + next.directions[0], node[1] + next.directions[1])
            if self.board_size <= neighbor[0] < len(self.grid)-self.board_size and self.board_size <= neighbor[1] < len(self.grid[0])-self.board_size:
                if self.grid[neighbor[0]][neighbor[1]] == 0:
                    if next.directions[0] != 0 and next.directions[1] != 0:  # 对角线方向
                        if self.grid[node[0] + next.directions[0]][node[1]] == 0 and self.grid[node[0]][node[1] + next.directions[1]] == 0:
                            neighbors.append(neighbor)
                            distances.append(next.cost)
                    else:
                        neighbors.append(neighbor)
                        distances.append(next.cost)

        return neighbors,distances

    def plan(self):
        open = []
        self.searched = [] # Used to record nodes that are searched
        g_cost = {self.start.position:0}

        self.start.h = self.heuristic(self.start.position)
        heapq.heappush(open, self.start)

        while open:
            # Select the node closest to the start node
            current_node = heapq.heappop(open)
            if current_node.position in self.searched:
                continue
            self.searched.append(current_node.position)
            
            # Find the goal
            if current_node == self.goal:
                self.goal = current_node
                self.path = []
                while current_node:
                    self.path.append(current_node.position)
                    current_node = current_node.parent
                self.path = self.path[::-1]
                        
                return self.goal.g
            
            # Find the neighbors of the current node and determine in turn if they have already been closed
            neighbors, distances = self.get_neighbors(current_node.position)
            for neighbor,distance in zip(neighbors,distances):
                if neighbor in self.searched:
                    continue
                h = self.heuristic(neighbor)
                g = current_node.g+distance

                if neighbor not in g_cost or g < g_cost[neighbor]:
                    g_cost[neighbor] = g
                    neighbor_node = self.Node(neighbor,g,h,current_node)
                    heapq.heappush(open,neighbor_node)
                
        return -1

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
        dx = abs(node[0] - self.goal.position[0])
        dy = abs(node[1] - self.goal.position[1])
        return D * math.sqrt(dx * dx + dy * dy)
    

    def visualize_grid(self, cost):
        fig, ax = plt.subplots()
        grid = np.array(self.grid)

        plt.imshow(grid, cmap='Greys', origin='upper')

        plt.title("AStar\n"+"Cost: "+str(cost))

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

            return visited_patches + [path_line]*10
    
        ani = animation.FuncAnimation(fig, update, frames=len(self.searched) + 10, interval=50, repeat=False)
        ani.save("map_animate.gif",writer='pillow')
        # ani.save("map_animate.mp4",writer='ffmpeg')

        plt.savefig("map_animate.png",bbox_inches='tight')
        plt.show()
        