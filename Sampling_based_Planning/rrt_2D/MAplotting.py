"""
Plotting tools for Sampling-based algorithms
@author: huiming zhou
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys
import env

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

# from Sampling_based_Planning.rrt_2D import env


class MAPlotting:
    def __init__(self, x1_start, x1_goal,x2_start, x2_goal,x3_start, x3_goal,x4_start, x4_goal):
        self.xI_1, self.xG_1 = x1_start, x1_goal
        self.xI_2, self.xG_2 = x2_start, x2_goal
        self.xI_3, self.xG_3 = x3_start, x3_goal
        self.xI_4, self.xG_4 = x4_start, x4_goal
        
        self.env = env.Env()
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle

    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def MAanimation(self, nodelist_1,nodelist_2,nodelist_3,nodelist_4, path_1,path_2,path_3,path_4, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist_1, animation)
        self.plot_visited(nodelist_2, animation)
        self.plot_visited(nodelist_3, animation)
        self.plot_visited(nodelist_4, animation)
        self.plot_path(path_1,path_2,path_3,path_4)
        # plt.show()

    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    def plot_grid(self, name):
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.xI_1[0], self.xI_1[1], "bs", linewidth=3)
        plt.plot(self.xG_1[0], self.xG_1[1], "gs", linewidth=3)
        plt.plot(self.xI_2[0], self.xI_2[1], "bs", linewidth=3)
        plt.plot(self.xG_2[0], self.xG_2[1], "gs", linewidth=3)
        plt.plot(self.xI_3[0], self.xI_3[1], "bs", linewidth=3)
        plt.plot(self.xG_3[0], self.xG_3[1], "gs", linewidth=3)
        plt.plot(self.xI_4[0], self.xI_4[1], "bs", linewidth=3)
        plt.plot(self.xG_4[0], self.xG_4[1], "gs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path_1,path_2,path_3,path_4):
        if len(path_1) != 0:
            plt.plot([x[0] for x in path_1], [x[1] for x in path_1], 'r-p', linewidth=2)
            # plt.pause(0.01)
        if len(path_2) != 0:
            plt.plot([x[0] for x in path_2], [x[1] for x in path_2], 'y-p', linewidth=2)
            # plt.pause(0.01)
        if len(path_3) != 0:
            plt.plot([x[0] for x in path_3], [x[1] for x in path_3], 'b-p', linewidth=2)
            # plt.pause(0.01)
        if len(path_4) != 0:
            plt.plot([x[0] for x in path_4], [x[1] for x in path_4], 'm-p', linewidth=2)
            # plt.pause(0.01)
        plt.show()
