"""
Plot tools 2D
@author: huiming zhou
"""

import os
import sys
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import env


class MAPlotting:
    def __init__(self, x1_start, x1_goal,x2_start, x2_goal,x3_start, x3_goal,x4_start, x4_goal):
        self.xI_1, self.xG_1 = x1_start, x1_goal
        self.xI_2, self.xG_2 = x2_start, x2_goal
        self.xI_3, self.xG_3 = x3_start, x3_goal
        self.xI_4, self.xG_4 = x4_start, x4_goal


        self.env = env.Env()
        self.obs = self.env.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def animation(self, path, visited, name):
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def MAanimation(self, visited_1,visited_2,visited_3,visited_4,path_1,path_2,path_3,path_4, name):
        self.plot_grid(name)
        # self.plot_visited1(visited_1)
        # self.plot_visited2(visited_2)
        # self.plot_visited3(visited_3)
        # self.plot_visited4(visited_4)
        self.plot_path(path_1,path_2,path_3,path_4)
        plt.show()


    def animation_lrta(self, path, visited, name):
        self.plot_grid(name)
        cl = self.color_list_2()
        path_combine = []

        for k in range(len(path)):
            self.plot_visited(visited[k], cl[k])
            plt.pause(0.2)
            self.plot_path(path[k])
            path_combine += path[k]
            plt.pause(0.2)
        if self.xI in path_combine:
            path_combine.remove(self.xI)
        self.plot_path(path_combine)
        plt.show()

    def animation_ara_star(self, path, visited, name):
        self.plot_grid(name)
        cl_v, cl_p = self.color_list()

        for k in range(len(path)):
            self.plot_visited(visited[k], cl_v[k])
            self.plot_path(path[k], cl_p[k], True)
            plt.pause(0.5)

        plt.show()

    def animation_bi_astar(self, path, v_fore, v_back, name):
        self.plot_grid(name)
        self.plot_visited_bi(v_fore, v_back)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name):
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        # plt.plot(self.xI_1[0], self.xI_1[1], "bs")
        # plt.plot(self.xG_1[0], self.xG_1[1], "gs")
        # plt.plot(self.xI_2[0], self.xI_2[1], "bs")
        # plt.plot(self.xG_2[0], self.xG_2[1], "gs")
        # plt.plot(self.xI_3[0], self.xI_3[1], "bs")
        # plt.plot(self.xG_3[0], self.xG_3[1], "gs")
        # plt.plot(self.xI_4[0], self.xI_4[1], "bs")
        # plt.plot(self.xG_4[0], self.xG_4[1], "gs")
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_visited1(self, visited, cl='gray'):
        if self.xI_1 in visited:
            visited.remove(self.xI_1)

        if self.xG_1 in visited:
            visited.remove(self.xG_1)        

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_visited2(self, visited, cl='gray'):
        if self.xI_2 in visited:
            visited.remove(self.xI_2)

        if self.xG_2 in visited:
            visited.remove(self.xG_2)        

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_visited3(self, visited, cl='gray'):
        if self.xI_3 in visited:
            visited.remove(self.xI_3)

        if self.xG_3 in visited:
            visited.remove(self.xG_3)        

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)
    
    def plot_visited4(self, visited, cl='gray'):
        if self.xI_4 in visited:
            visited.remove(self.xI_4)

        if self.xG_4 in visited:
            visited.remove(self.xG_4)        

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40
            #
            # length = 15

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)


    def plot_path(self, path_1,path_2,path_3,path_4, cl='r', flag=False):
        path_x_1 = [path_1[i][0] for i in range(len(path_1))]
        path_y_1 = [path_1[i][1] for i in range(len(path_1))]
        path_x_2 = [path_2[i][0] for i in range(len(path_2))]
        path_y_2 = [path_2[i][1] for i in range(len(path_2))]
        path_x_3 = [path_3[i][0] for i in range(len(path_3))]
        path_y_3 = [path_3[i][1] for i in range(len(path_3))]
        path_x_4 = [path_4[i][0] for i in range(len(path_4))]
        path_y_4 = [path_4[i][1] for i in range(len(path_4))]


        if not flag:
            plt.plot(path_x_1, path_y_1, linewidth='3', color='r')
            plt.plot(path_x_2, path_y_2, linewidth='3', color='y')
            plt.plot(path_x_3, path_y_3, linewidth='3', color='g')
            plt.plot(path_x_4, path_y_4, linewidth='3', color='b')
        else:
            plt.plot(path_x_1, path_y_1, linewidth='3', color=cl)

        plt.plot(self.xI_1[0], self.xI_1[1], "bs")
        plt.plot(self.xG_1[0], self.xG_1[1], "gs")
        plt.plot(self.xI_2[0], self.xI_2[1], "bs")
        plt.plot(self.xG_2[0], self.xG_2[1], "gs")
        plt.plot(self.xI_3[0], self.xI_3[1], "bs")
        plt.plot(self.xG_3[0], self.xG_3[1], "gs")
        plt.plot(self.xI_4[0], self.xI_4[1], "bs")
        plt.plot(self.xG_4[0], self.xG_4[1], "gs")


        plt.pause(0.01)

    def plot_visited_bi(self, v_fore, v_back):
        if self.xI in v_fore:
            v_fore.remove(self.xI)

        if self.xG in v_back:
            v_back.remove(self.xG)

        len_fore, len_back = len(v_fore), len(v_back)

        for k in range(max(len_fore, len_back)):
            if k < len_fore:
                plt.plot(v_fore[k][0], v_fore[k][1], linewidth='3', color='gray', marker='o')
            if k < len_back:
                plt.plot(v_back[k][0], v_back[k][1], linewidth='3', color='cornflowerblue', marker='o')

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 10 == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_v, cl_p

    @staticmethod
    def color_list_2():
        cl = ['silver',
              'steelblue',
              'dimgray',
              'cornflowerblue',
              'dodgerblue',
              'royalblue',
              'plum',
              'mediumslateblue',
              'mediumpurple',
              'blueviolet',
              ]
        return cl
