"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import env, utils,MAplotting
# from MAplotting import MAplotting

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")
# from Sampling_based_Planning.rrt_2D.MAplotting import MAplotting
# from Sampling_based_Planning.rrt_2D import env, plotting, utils


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class MARRT:
    def __init__(self, M_start, M_goal, step_len, goal_sample_rate, iter_max):
        
        # define MARRT start and goal point
        self.s1_start = Node(M_start[0])
        self.s1_goal = Node(M_goal[0])
        self.s2_start = Node(M_start[1])
        self.s2_goal = Node(M_goal[1])
        self.s3_start = Node(M_start[2])
        self.s3_goal = Node(M_goal[2])
        self.s4_start = Node(M_start[3])
        self.s4_goal = Node(M_goal[3])
        self.s_start = [Node(M_start[0]), Node(M_start[1]),Node(M_start[2]),Node(M_start[3])]
        self.s_goal = [Node(M_goal[0]), Node(M_goal[1]),Node(M_goal[2]),Node(M_goal[3])]

        self.goal_arrived = [0,0,0,0]
        self.goal_extended = [1,1,1,1]
        self.failure_rate = [0,0,0,0]
        

        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex_1 = [self.s1_start]
        self.vertex_2 = [self.s2_start]
        self.vertex_3 = [self.s3_start]
        self.vertex_4 = [self.s4_start]
        self.vertex = [self.s_start]
        self.path1 = []
        self.path2 = []
        self.path3 = []
        self.path4 = []

        self.env = env.Env()
        self.plotting_1 = MAplotting.MAPlotting(M_start[0], M_goal[0],M_start[1], M_goal[1],M_start[2], M_goal[2],M_start[3], M_goal[3])
        # self.plotting_2 = plotting.Plotting(M_start[1], M_goal[1])
        # self.plotting_3 = plotting.Plotting(M_start[2], M_goal[2])
        # self.plotting_4 = plotting.Plotting(M_start[3], M_goal[3])

        # self.MAplotting = plotting.Plotting(self.s4_start, self.s4_goal)
        self.utils = utils.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            
            node_rand = self.generate_random_node(self.goal_sample_rate)
            # print("fxxk:\\",i,self.vertex)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.steer(node_near, node_rand)
            # print("iter:\\",i,self.vertex)
            # path = [0,0,0,0]
            if node_new and not self.utils.is_collision(node_near[0], node_new[0])and not self.utils.is_collision(node_near[1], node_new[1])and not self.utils.is_collision(node_near[2], node_new[2])and not self.utils.is_collision(node_near[3], node_new[3]):
                self.vertex.append(node_new)
                self.vertex_1.append(node_new[0])
                # self.vertex_2.append(node_new[1])
                # self.vertex_3.append(node_new[2])
                # self.vertex_4.append(node_new[3])
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)
                # print(dist)
                for j in range (4):
                    if dist[j] <= self.step_len and not self.utils.is_collision(node_new[j], self.s_goal[j]):
                        self.goal_arrived[j] = 1
                        # self.steer(node_new, self.s_goal)
                        # print(node_new[j].x,node_new[j].y)
                        # self.path[j] = self.extract_path(node_new[j], self.s_goal[j])
                        # print(self.path[j])
                        print("iter:\\",i,self.goal_arrived)
                        self.path1 = self.extract_path(node_new[0], self.s_goal[0])
                        self.path2 = self.extract_path(node_new[1], self.s_goal[1])
                        self.path3 = self.extract_path(node_new[2], self.s_goal[2])
                        self.path4 = self.extract_path(node_new[3], self.s_goal[3])


        return [self.path1,self.path2,self.path3,self.path4]
        # return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta
        node_rand = [0,0,0,0]
        if np.random.random() > goal_sample_rate and self.goal_arrived != [1,1,1,1]:
            for i in range (4):
                if self.goal_arrived[i] == 1:
                    node_rand[i]= self.s_goal[i]
                else:
                    node_rand[i] = Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))
    
            return node_rand
        return self.s_goal

    # @staticmethod
    def nearest_neighbor(self, node_list, n):

        # print(f"list: {node_list[:][0]}")
        temp = [[], [], [], []]
        # print(temp)
        # temp[0].append(Node((0, 0)))
        # print(temp)
        # print(self.vertex_1)
        # print(node_list)
        for nd in node_list:
            # print("nd:\\",nd.x,nd.y)
            for i in range(4):
                temp[i].append(nd[i])
                # print('ndi:\\',nd[i])
                # print('tempi:\\',temp[i])
        # print(temp)

        min_index_1 = int(np.argmin([math.hypot(nd.x - n[0].x, nd.y - n[0].y) for nd in temp[0]]))
        min_index_2 = int(np.argmin([math.hypot(nd.x - n[1].x, nd.y - n[1].y) for nd in temp[1]]))
        min_index_3 = int(np.argmin([math.hypot(nd.x - n[2].x, nd.y - n[2].y) for nd in temp[2]]))
        min_index_4 = int(np.argmin([math.hypot(nd.x - n[3].x, nd.y - n[3].y) for nd in temp[3]]))

        return [node_list[min_index_1][0], node_list[min_index_2][1], node_list[min_index_3][2], node_list[min_index_4][3]]
        # return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
        #                                 for nd in node_list]))]


    def steer(self, node_start, node_end):
    
        dist, theta = self.get_distance_and_angle(node_start, node_end)
        node_new = [Node((0, 0)), Node((0, 0)), Node((0, 0)), Node((0, 0))]
        for i in range (4):
            dist[i] = min(self.step_len, dist[i])
            node_new[i] = Node((node_start[i].x + dist[i] * math.cos(theta[i]),
                         node_start[i].y + dist[i] * math.sin(theta[i])))
            node_new[i].parent = node_start[i]

        return node_new

    def extract_path(self, node_end, s_goal):
        path = [(s_goal.x, s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    # @staticmethod
    def get_distance_and_angle(self, node_start, node_end):
        dx = [0, 0, 0, 0]
        dy = [0, 0, 0, 0]
        for i in range (4):
            dx[i] = node_end[i].x - node_start[i].x
            dy[i] = node_end[i].y - node_start[i].y

        list_1 = [math.hypot(dx[0], dy[0]), math.hypot(dx[1], dy[1]),math.hypot(dx[2], dy[2]),math.hypot(dx[3], dy[3])]
        list_2 = [math.atan2(dy[0], dx[0]), math.atan2(dy[1], dx[1]),math.atan2(dy[2], dx[2]),math.atan2(dy[3], dx[3])]
        return list_1, list_2

def main():
    x1_start = (2, 2)  # Starting node
    x1_goal = (48, 23)  # Goal node
    x2_start = (3, 2)  # Starting node
    x2_goal = (49, 23)  # Goal node
    x3_start = (3, 3)  # Starting node
    x3_goal = (49, 24)  # Goal node
    x4_start = (2, 3)  # Starting node
    x4_goal = (48, 24)  # Goal node
    M_start = [x1_start,x2_start,x3_start,x4_start]
    M_goal = [x1_goal,x2_goal,x3_goal,x4_goal]
    
    marrt = MARRT(M_start, M_goal, 1, 0.05, 3000)
    path = marrt.planning()
    # print(marrt.vertex_1)
    if path:
        marrt.plotting_1.MAanimation(marrt.vertex_1, marrt.vertex_2,marrt.vertex_3,marrt.vertex_4, path[0],path[1],path[2],path[3], "RRT", False)
        # marrt.plotting_2.animation(marrt.vertex_2, path[1], "RRT", False)
        # marrt.plotting_3.animation(marrt.vertex_3, path[2], "RRT", False)
        # marrt.plotting_4.animation(marrt.vertex_4, path[3], "RRT", False)
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()
