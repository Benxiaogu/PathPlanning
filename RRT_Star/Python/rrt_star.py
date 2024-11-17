"""
  @File: rrt_star.py
  @Brief: RRT Star algorithm for pathplanning
 
  @Author: Benxiaogu
  @Github: https://github.com/Benxiaogu
  @CSDN: https://blog.csdn.net/weixin_51995147?type=blog
 
  @Date: 2024-11-17
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from RRT.Python.rrt import RRT


class RRTStar(RRT):
    def __init__(self,start,goal,obstacles,search_r,board_size,max_try,max_dist,goal_sample_rate,env) -> None:
        super().__init__(start,goal,obstacles,board_size,max_try,max_dist,goal_sample_rate,env)
        self.search_r = search_r # radius of searching new parent

    def __str__(self) -> str:
        return "RRT*"


    def plan(self):
        self.searched.append(self.start)
        closed_list = {self.start.position: self.start}
        # plan max_try times
        for i in range(self.max_try):
            node_rand = self.get_random_node()
            # visited
            if node_rand.position in closed_list:
                continue
            # Get the nearest neighbor node
            # node_near = self.get_nearest_neighbor(list(closed_list.values()),node_rand)
            # Get the new node
            node_new = self.get_new_node(node_rand,list(closed_list.values()))
            if node_new:
                closed_list[node_new.position] = node_new
                self.searched.append(node_new)
                dist = self.distance(node_new,self.goal)
                # Found goal successfully
                if dist <= self.max_dist and not self.isCollision(node_new,self.goal):
                    self.searched.append(self.goal)
                    self.goal.parent = node_new.position
                    self.goal.cost = node_new.cost + self.distance(self.goal,node_new)
                    closed_list[self.goal.position] = self.goal
                    cost, path= self.extractPath(closed_list)

                    print("Exploring {} nodes.".format(i))
                    return cost,path
                
        return 0,None
    
    
    def get_new_node(self, node_rand, node_list):
        dist = [self.distance(node_rand, n) for n in node_list]
        node_near = node_list[int(np.argmin(dist))]
    
        node_new = super().get_new_node(node_rand, node_near)
        if node_new:
            for node_n in node_list:
                new_dist = self.distance(node_n,node_new)
                if new_dist < self.search_r and not self.isCollision(node_n, node_new):
                    print("---------------------------------------------")
                    print("new_dist: ",new_dist)
                    print("isCollision: ",self.isCollision(node_n, node_new))
                    print("---------------------------------------------")
                    cost = node_n.cost + new_dist
                    if node_new.cost > cost:
                        # find who is the best parent of node_new
                        node_new.parent = node_n.position
                        node_new.cost = cost
                    else:
                        cost = node_new.cost + new_dist
                        if node_n.cost > cost:
                            # find whose best parent is node_new
                            node_n.parent = node_new.position
                            node_n.cost = cost
                else:
                    continue
            return node_new
        else:
            return None
