"""
Environment for rrt_2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        self.x_range = (0, 80)
        self.y_range = (0, 50)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 30, 1],
            [1, 0, 30, 1],
            [30, 1, 1, 30],
            [14, 6, 2, 18],
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [21, 5, 2, 2],

        ]
        # obs_rectangle = []
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [21, 20, 1.5],
        ]

        return obs_cir
