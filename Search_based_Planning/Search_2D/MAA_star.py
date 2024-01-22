"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import env, MAplotting
# from Search_2D import MAplotting as Plotting


class MAAStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, M_start, M_goal, heuristic_type):

        self.s1_start = M_start[0]
        self.s1_goal = M_goal[0]
        self.s2_start = M_start[1]
        self.s2_goal = M_goal[1]
        self.s3_start = M_start[2]
        self.s3_goal = M_goal[2]
        self.s4_start = M_start[3]
        self.s4_goal = M_goal[3]
        self.s_start = [M_start[0], M_start[1],M_start[2],M_start[3]]
        self.s_goal = [M_goal[0], M_goal[1],M_goal[2],M_goal[3]]

        self.goal_arrived = [0,0,0,0]
        self.goal_extended = [1,1,1,1]
        self.failure_rate = [0,0,0,0]

        self.vertex_1 = [self.s1_start]
        self.vertex_2 = [self.s2_start]
        self.vertex_3 = [self.s3_start]
        self.vertex_4 = [self.s4_start]
        self.vertex = [self.s_start]
        self.path1 = []
        self.path2 = []
        self.path3 = []
        self.path4 = []

        # self.s_start = s_start
        # self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN_1 = []  # priority queue / OPEN set
        self.CLOSED_1 = []  # CLOSED set / VISITED order
        self.OPEN_2 = []  # priority queue / OPEN set
        self.CLOSED_2 = []
        self.OPEN_3 = []  # priority queue / OPEN set
        self.CLOSED_3 = []
        self.OPEN_4 = []  # priority queue / OPEN set
        self.CLOSED_4 = []
        
        self.PARENT_1 = dict()  # recorded parent
        self.PARENT_2 = dict()
        self.PARENT_3 = dict()
        self.PARENT_4 = dict()

        # self.g = dict()  # cost to come
        self.g_1 = dict()
        self.g_2 = dict()
        self.g_3 = dict()
        self.g_4 = dict()
        # print(self.g_1)




    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT_1[self.s1_start] = self.s1_start
        self.PARENT_2[self.s2_start] = self.s2_start
        self.PARENT_3[self.s3_start] = self.s3_start
        self.PARENT_4[self.s4_start] = self.s4_start



        self.g_1[self.s1_start] = 0
        self.g_1[self.s1_goal] = math.inf
        self.g_2[self.s2_start] = 0
        self.g_2[self.s2_goal] = math.inf
        self.g_3[self.s3_start] = 0
        self.g_3[self.s3_goal] = math.inf
        self.g_4[self.s4_start] = 0
        self.g_4[self.s4_goal] = math.inf  


        heapq.heappush(self.OPEN_1,(self.f_value_1(self.s1_start), self.s1_start))
        heapq.heappush(self.OPEN_2,(self.f_value_2(self.s2_start), self.s2_start))
        heapq.heappush(self.OPEN_3,(self.f_value_3(self.s3_start), self.s3_start))
        heapq.heappush(self.OPEN_4,(self.f_value_4(self.s4_start), self.s4_start))

        while self.OPEN_1 and self.OPEN_2 and self.OPEN_3 and self.OPEN_4:
            _, s_1 = heapq.heappop(self.OPEN_1)
            self.CLOSED_1.append(s_1)
            _, s_2 = heapq.heappop(self.OPEN_2)
            self.CLOSED_2.append(s_2)
            _, s_3 = heapq.heappop(self.OPEN_3)
            self.CLOSED_3.append(s_3)
            _, s_4 = heapq.heappop(self.OPEN_4)
            self.CLOSED_4.append(s_4)

            # s_tmp = [s_1,s_2,s_3,s_4]

            # self.CLOSED.append(s)
            # if s_tmp == self.s_goal:  # stop condition
            #     break
            if s_1 == self.s1_goal and s_2 == self.s2_goal and s_3 == self.s3_goal and s_4 == self.s4_goal:  # stop condition
                break
            # print("s1:",s_1,"s2",s_2,"s3:",s_3,"s4",s_4,"\n")
            
            for s_n1 in self.get_neighbor(s_1):
                new_cost_1 = self.g_1[s_1] + self.cost(s_1, s_n1,s_n1,s_2,s_3,s_4)
                # new_cost_1 = self.g_1[s_1] + self.cost(s_1, s_n1)
                if s_n1 not in self.g_1:
                    self.g_1[s_n1] = math.inf
                # print("s_1 out:",s_1,"\n")
                # if new_cost_1 < self.g_1[s_n1] and np.linalg.norm(np.array(s_n1)-np.array(s_2)) < 10 and np.linalg.norm(np.array(s_n1)-np.array(s_3)) < 10 and np.linalg.norm(np.array(s_n1)-np.array(s_4)) < 10:
                if new_cost_1< self.g_1[s_n1]:
                    # print("s_1 in :",s_1)
                    self.g_1[s_n1] = new_cost_1
                    self.PARENT_1[s_n1] = s_1
                    heapq.heappush(self.OPEN_1, (self.f_value_1(s_n1), s_n1))
            
            for s_n2 in self.get_neighbor(s_2):
                # new_cost_2 = self.g_2[s_2] + self.cost(s_2, s_n2)
                new_cost_2 = self.g_2[s_2] + self.cost(s_2, s_n2,s_n2,s_1,s_3,s_4)

                if s_n2 not in self.g_2:
                    self.g_2[s_n2] = math.inf
                # if new_cost_2 < self.g_2[s_n2] and np.linalg.norm(np.array(s_n2)-np.array(s_1)) < 10 and np.linalg.norm(np.array(s_n2)-np.array(s_3)) < 10 and np.linalg.norm(np.array(s_n2)-np.array(s_4)) < 10:
                if new_cost_2 < self.g_2[s_n2]:
                    self.g_2[s_n2] = new_cost_2
                    self.PARENT_2[s_n2] = s_2
                    heapq.heappush(self.OPEN_2, (self.f_value_2(s_n2), s_n2))
            
            for s_n3 in self.get_neighbor(s_3):
                # new_cost_3 = self.g_3[s_3] + self.cost(s_3, s_n3)
                new_cost_3 = self.g_3[s_3] + self.cost(s_3, s_n3,s_n3,s_1,s_2,s_4)
                if s_n3 not in self.g_3:
                    self.g_3[s_n3] = math.inf
                # if new_cost_3 < self.g_3[s_n3] and np.linalg.norm(np.array(s_n3)-np.array(s_1)) < 10 and np.linalg.norm(np.array(s_n3)-np.array(s_2)) < 10 and np.linalg.norm(np.array(s_n3)-np.array(s_4)) < 10:
                if new_cost_3 < self.g_3[s_n3]:
                    self.g_3[s_n3] = new_cost_3
                    self.PARENT_3[s_n3] = s_3
                    heapq.heappush(self.OPEN_3, (self.f_value_3(s_n3), s_n3))

            for s_n4 in self.get_neighbor(s_4):
                # new_cost_4 = self.g_4[s_4] + self.cost(s_4, s_n4)
                new_cost_4 = self.g_4[s_4] + self.cost(s_4, s_n4,s_n4,s_1,s_2,s_3)
                if s_n4 not in self.g_4:
                    self.g_4[s_n4] = math.inf
                # if new_cost_4 < self.g_4[s_n4] and np.linalg.norm(np.array(s_n4)-np.array(s_1)) < 10 and np.linalg.norm(np.array(s_n4)-np.array(s_2)) < 10 and np.linalg.norm(np.array(s_n4)-np.array(s_3)) < 10:
                if new_cost_4 < self.g_4[s_n4]:
                    self.g_4[s_n4] = new_cost_4
                    self.PARENT_4[s_n4] = s_4
                    heapq.heappush(self.OPEN_4, (self.f_value_4(s_n4), s_n4))
            
        



        return self.extract_path(self.PARENT_1,self.s1_start,self.s1_goal), self.CLOSED_1,self.extract_path(self.PARENT_2,self.s2_start,self.s2_goal), self.CLOSED_2,self.extract_path(self.PARENT_3,self.s3_start,self.s3_goal), self.CLOSED_3,self.extract_path(self.PARENT_4,self.s4_start,self.s4_goal), self.CLOSED_4

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal,s,s_1,s_2,s_3):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf
        print("dist:",np.linalg.norm(np.array(s)-np.array(s_1)),"\n")
        if np.linalg.norm(np.array(s)-np.array(s_1)) > 10 or np.linalg.norm(np.array(s)-np.array(s_1))< 2:
            return math.inf
        if np.linalg.norm(np.array(s)-np.array(s_2)) > 10 or np.linalg.norm(np.array(s)-np.array(s_2))< 2:
            return math.inf
        if np.linalg.norm(np.array(s)-np.array(s_3)) > 10 or np.linalg.norm(np.array(s)-np.array(s_3))< 2:
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value_1(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_1[s] + self.heuristic(s,self.s1_goal)
    

    def f_value_2(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_2[s] + self.heuristic(s,self.s2_goal)
    
    def f_value_3(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_3[s] + self.heuristic(s,self.s3_goal)
    
    def f_value_4(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_4[s] + self.heuristic(s,self.s4_goal)

    def extract_path(self, PARENT, s_start, s_goal):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [s_goal]
        s = s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == s_start:
                break

        return list(path)

    def heuristic(self, s, s_goal):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])


def main():
    # s_start = (5, 5)
    # s_goal = (45, 25)
    x1_start = (5, 5)  # Starting node
    x1_goal = (40, 40)  # Goal node
    x2_start = (10, 5)  # Starting node
    x2_goal = (45, 40)  # Goal node
    x3_start = (10, 10)  # Starting node
    x3_goal = (45, 45)  # Goal node
    x4_start = (5, 10)  # Starting node
    x4_goal = (40, 45)  # Goal node
    M_start = [x1_start,x2_start,x3_start,x4_start]
    M_goal = [x1_goal,x2_goal,x3_goal,x4_goal]
    astar = MAAStar(M_start, M_goal, "euclidean")
    plot = MAplotting.MAPlotting(x1_start,x1_goal,x2_start,x2_goal,x3_start,x3_goal,x4_start,x4_goal)

    path_1, visited_1,path_2, visited_2,path_3, visited_3,path_4, visited_4 = astar.searching()
    plot.MAanimation(visited_1, visited_2,visited_3,visited_4,path_1,path_2,path_3,path_4, "A*")  # animation

    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()
