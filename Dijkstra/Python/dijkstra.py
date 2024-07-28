import heapq
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Dijkstra:
    def __init__(self,grid,start,goal,radius,board_size):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.radius = radius
        self.board_size = board_size

    class Node:
        def __init__(self,directions,cost):
            self.directions = directions
            self.cost = cost
    
    def get_neighbors(self, node):
        neighbors = []
        distances = []
        nexts = [self.Node((-1, 0),1), self.Node((0, 1),1), self.Node((0, -1),1), self.Node((1,0),1),
                self.Node((-1,1),math.sqrt(2)), self.Node((1,1),math.sqrt(2)),self.Node((1, -1),math.sqrt(2)), self.Node((-1,-1),math.sqrt(2))]
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
        priority_queue = []
        heapq.heappush(priority_queue,(0,self.start))
        costs = {self.start: 0}
        previous_nodes = {self.start: None}
        self.visited = set()
        self.visit_order = []

        while priority_queue:
            current_cost, current_node = heapq.heappop(priority_queue)
            # Determines whether the current node has already been visited
            if current_node in self.visited:
                continue

            self.visited.add(current_node)
            self.visit_order.append(current_node)
            if current_node == self.goal:
                break
            # Find passable neighbors
            neighbors, distances = self.get_neighbors(current_node)
            for neighbor,distance in zip(neighbors,distances):
                if neighbor[0] == -1:
                    continue
                # Compute the cost from the start point
                cost = current_cost + distance
                # Store cost and update to minimum value
                if neighbor not in costs or cost < costs[neighbor]:
                    costs[neighbor] = cost
                    # The parent node of neighbor is current_node
                    previous_nodes[neighbor] = current_node
                    # Push the node into priority_queue
                    heapq.heappush(priority_queue,(cost, neighbor))
        
        self.path = []
        current_node = self.goal
        while current_node is not None:
            self.path.append(current_node)
            current_node = previous_nodes.get(current_node)
        self.path = self.path[::-1]

        return costs[self.goal]

    def visualize_grid(self, cost):
        fig, ax = plt.subplots()
        grid = np.array(self.grid)

        plt.imshow(grid, cmap='Greys', origin='upper')

        # Mark the start and goal point
        plt.scatter(self.start[1], self.start[0], c='green', marker='o', s=60)
        plt.scatter(self.goal[1], self.goal[0], c='blue', marker='o', s=60)

        # # Mark locations which has been visited
        # for node in self.visited:
        #     plt.gca().add_patch(plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5))
            
        # # 标记路径
        # if self.path:
        #     path_x, path_y = zip(*self.path)
        #     plt.plot(path_y, path_x, c='red', linewidth=2)

        visited_patches = []
        path_line, = ax.plot([], [], c='red', linewidth=2)
        for order in range(len(self.visit_order)):
            node = self.visit_order[order]
            plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5)

        def update(frame):
            if frame < len(self.visit_order):
                node = self.visit_order[frame]
                patch = plt.Rectangle((node[1]-0.5, node[0]-0.5), 1, 1, fill=True, color='gray', alpha=0.5)
                visited_patches.append(patch)
                ax.add_patch(patch)
            else:
                path_x, path_y = zip(*self.path)
                path_line.set_data(path_y, path_x)

            return visited_patches + [path_line]*10
    
        plt.title("Dijkstra\n"+"Cost: "+str(cost))
        ani = animation.FuncAnimation(fig, update, frames=len(self.visit_order) + 10, interval=10, repeat=False)
        ani.save("map_animate.mp4",writer='ffmpeg')

        plt.savefig("map_animate.png",bbox_inches='tight')
        plt.show()
