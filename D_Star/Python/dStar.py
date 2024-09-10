"""
    Filename: dStar.py
    Description: Plan path using Dynamic A* Algorithm
    Author: Benxiaogu:https://github.com/Benxiaogu
    Date: 2024-08-22
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Node:
    """
    Class for D* nodes.

    Parameters:
        position (tuple): current coordinate
        parent (tuple): coordinate of parent node
        t (str): state of node, including `NEW` `OPEN` and `CLOSED`
                    'NEW': the node was never put into openlist;
                    'OPEN': the node is in openlist
                    'CLOSED': the node is no longer in openlist
        h (float): cost from goal to current node
        k (float): minimum cost from goal to current node in history
    """
    def __init__(self, position, parent, t, h, k) -> None:
        self.position = position
        self.parent = parent
        self.t = t
        self.h = h
        self.k = k

    def __add__(self, node):
        return Node((self.x + node.x, self.y + node.y), 
                     self.parent, self.t, self.h + node.h, self.k)
    
    def __eq__(self, other) -> bool:
        return self.position == other.position
        
    def __lt__(self,other):
        return self.k+self.h < other.k+other.h or (self.k+self.h==other.k+other.h and self.h<other.h)

class DStar:
    def __init__(self,grid,start,goal,board_size) -> None:
        self.grid = grid
        self.start = Node(start,None,'NEW',float('inf'),float('inf'))
        self.goal = Node(goal,None,'NEW',0,float('inf'))
        self.board_size = board_size
        self.path = []
        self.open = []
        self.searched = [] # Used to record nodes that are searched

        grid_map = []
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                grid_map.append((i,j))

        self.map = {n: Node(n, None, 'NEW', float('inf'), float('inf')) for n in grid_map}
        self.map[self.goal.position] = self.goal
        self.map[self.start.position] = self.start
        self.insert(self.goal,0)

        self.fig, self.ax = plt.subplots()

    def run(self):
        cost = self.plan()
        self.fig.canvas.mpl_connect('button_press_event', self.update)
        self.visualize_grid(cost)

    def plan(self):
        while True:
            self.process_state()
            if self.start.t == 'CLOSED':
                break
        self.searched.clear()
        # Find the goal
        pathnode = self.start
        cost = 0
        self.path.append(pathnode.position)
        while pathnode != self.goal:
            nodeparent = self.map[pathnode.parent]
            cost += self.cost(pathnode, nodeparent)
            pathnode = nodeparent
            self.path.append(pathnode.position)
            
        return cost

    class Neighbor:
        def __init__(self,direction,cost):
            self.direction = direction
            self.cost = cost
    
    def get_neighbors(self, current_node):
        neighbors = []
        distances = []
        node = current_node.position
        nexts = [self.Neighbor((-1, 0),1), self.Neighbor((0, 1),1), self.Neighbor((0, -1),1), self.Neighbor((1,0),1),
                self.Neighbor((-1,1),math.sqrt(2)), self.Neighbor((1,1),math.sqrt(2)),self.Neighbor((1, -1),math.sqrt(2)), self.Neighbor((-1,-1),math.sqrt(2))]
        for next in nexts:
            neighbor = (node[0] + next.direction[0], node[1] + next.direction[1])
            neighborNode = self.map[neighbor]
            if not self.isCollision(node,neighbor):
                neighbors.append(neighborNode)
                distances.append(next.cost)

        return neighbors,distances
    
    def isCollision(self,node1,node2):
        """
            Check if there will be a collision from node1 to node2
        """
        if self.board_size <= node1[0] < len(self.grid)-self.board_size \
            and self.board_size <= node1[1] < len(self.grid[0])-self.board_size \
                and self.board_size <= node2[0] < len(self.grid)-self.board_size \
                    and self.board_size <= node2[1] < len(self.grid[0])-self.board_size:
            if self.grid[node1[0]][node1[1]] == 0 and self.grid[node2[0]][node2[1]] == 0:
                direction1 = node1[0]-node2[0]
                direction2 = node1[1]-node2[1]
                if direction1 != 0 and direction2 != 0:  # 对角线方向
                    if self.grid[node1[0]][node2[1]] == 0 and self.grid[node2[0]][node1[1]] == 0:
                        return False
                else:
                    return False

        return True

    def process_state(self):
        """
            Compute optimal path costs to the goal
        """
        node =self.min_state # Find the node whose value of node.k is the smallest.
        self.searched.append(node.position)

        if node is None:
            return -1

        k_old = self.get_kmin # Find the minimum node.k as k_old
        self.delete(node) # Delete the node from openlist, and modify node.t from 'OPEN' to 'CLOSE'
        
        neighbors, distances = self.get_neighbors(node)
        # RASIE State, its path cost may not be optimal
        if k_old < node.h:                                                                              #
            for neighbor,distance in zip(neighbors,distances):                                          #
                if neighbor.h <= k_old and node.h > neighbor.h + distance:                              #
                    node.parent = neighbor.position # Added obstacles, replanning, find better neighbor #
                    node.h = neighbor.h + distance                                                      # 检查node的最优相邻状态，如果存在则降低 h 值
                    print("k_old<node.h")
        # LOWER State, its path cost is optimal since h ( X ) is equal to the old k_min .
        if k_old == node.h:
            for neighbor,distance in zip(neighbors,distances):
                if neighbor.t == 'NEW' or (neighbor.parent == node.position and neighbor.h != node.h+distance) \
                    or (neighbor.parent != node.position and neighbor.h > node.h+distance):
                    neighbor.parent = node.position # Searching commonly
                    self.insert(neighbor,node.h+distance)
                    print("k_old==node.h")
        else:
            for neighbor,distance in zip(neighbors,distances):
                if neighbor.t == 'NEW' or (neighbor.parent == node.position and neighbor.h != node.h+distance):
                    neighbor.parent = node.position
                    self.insert(neighbor,node.h+distance)
                else:
                    if neighbor.parent != node.position and neighbor.h > node.h + distance:
                        self.insert(node,node.h)
                    else:
                        if neighbor.parent != node.position and node.h > neighbor.h+distance \
                            and neighbor.t == 'CLOSED' and neighbor.h > k_old:
                            self.insert(neighbor,neighbor.h)
            print("else")

        return self.get_kmin

    @property
    def min_state(self):
        """
            Return the node who possesses the minimum value of k.
        """
        if not self.open:
            return None
        min_state = min(self.open, key=lambda x: x.k)
        return min_state
    
    @property
    def get_kmin(self):
        """
            Return the minimum value of k.
        """
        return self.min_state.k

    def delete(self, node) -> None:
        """
            Remove node from openlist, and change node.t from 'OPEN' to 'CLOSED'
        """
        if node.t == 'OPEN':
            node.t = 'CLOSED'
        self.open.remove(node)

    def insert(self, node: Node, h_new: float) -> None:
        """
            Modify the state of node as well as the node.h value and node.k value.
        """
        if node.t == 'NEW':         node.k = h_new
        elif node.t == 'OPEN':      node.k = min(node.k, h_new)
        elif node.t == 'CLOSED':    node.k = min(node.h, h_new)
        node.h = h_new
        node.t = 'OPEN'
        self.open.append(node)

    def modify_cost(self,node,node_parent):
        """
            Change the cost and enter affected nodes on the OPEN list.
        """
        if node.t == 'CLOSED':
            self.insert(node,node_parent.h+self.cost(node,node_parent))
        while True:
            k_min = self.process_state()
            # print("k_min: {} , node.h: {}\n".format(k_min,node.h))
            if k_min >= node.h:
                break

    def cost(self, node1: Node, node2: Node) -> float:
        """
            Return Euclidean distance if there is no collision from node1 to node2 otherwise 'inf'
        """
        if self.isCollision(node1.position, node2.position):
            return float("inf")
        return math.hypot(node2.position[0] - node1.position[0], node2.position[1] - node1.position[1])

    def update(self, event) -> None:
        """
            Update obstacles and replan from affected node.
        """
        x, y = int(event.xdata), int(event.ydata)
        if y < self.board_size or y > len(self.grid) -self.board_size- 1 or x < self.board_size or x > len(self.grid[0])-self.board_size - 1:
            print("Please choose right area!")
        else:
            self.searched.clear()
            if self.grid[y][x] == 0:
                # update obstacles
                self.grid[y][x] = 1

                node = self.start
                cost = 0
                self.path.clear()
                while node != self.goal:
                    node_parent = self.map[node.parent]
                    if self.isCollision(node.position, node_parent.position): 
                        # when meeting collisions, call modify_cost() function
                        self.modify_cost(node,node_parent)
                        continue
                    self.path.append(node.position)
                    cost += self.cost(node, node_parent)
                    node = node_parent
                self.path.append(self.goal.position)

                plt.cla()
                self.visualize_grid(cost)

    def visualize_grid(self, cost):
        grid = np.array(self.grid)

        plt.imshow(grid, cmap='Greys', origin='upper')

        plt.title("DStar\n"+"Cost: "+str(cost))

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
        path_line, = self.ax.plot([], [], c='red', linewidth=2)
        # for order in range(len(self.searched)):
        #     node = self.searched[order]
        #     plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5)

        def update(frame):
            if frame < len(self.searched):
                node = self.searched[frame]
                patch = plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5)
                visited_patches.append(patch)
                self.ax.add_patch(patch)
            elif self.path:
                path_x, path_y = zip(*self.path)
                path_line.set_data(path_y, path_x)

            return visited_patches + [path_line]*10
    
        ani = animation.FuncAnimation(self.fig, update, frames=len(self.searched) + 10, interval=100, repeat=False)
        ani.save("map_animate.gif",writer='pillow')
        # ani.save("map_animate.mp4",writer='ffmpeg')

        plt.savefig("map_animate.png",bbox_inches='tight')
        plt.show()
