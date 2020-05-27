import numpy as np
import matplotlib.pyplot as plt
from rrt import *
# from rrt_star.py import *   # TODO: Add optimization for star algorithm


def RRTsolver():

    testing = True

    world_size = 10
    eps = 1.0
    it = 2000

    #------------------------------------------------------------------------------------------------------#

    OBSTACLES = np.array([  # Obstacles are rectangles made out of line segments defined by 2 points each

        ((1, 3), (2, 3)),   ####
        ((2, 3), (2, 4)),   # Square of length 1
        ((1, 4), (2, 4)),   #
        ((1, 3), (1, 4)),   ####

        ((4, 4), (5, 4)),   ####
        ((5, 4), (5, 5)),   # Rectangle 2x4
        ((4, 5), (5, 5)),   #
        ((4, 4), (4, 5))    ####

    ])

    x_init = [1,1]
    x_goal = [9,9]

    grrt = GeometricRRT([0,0], [world_size, world_size], x_init, x_goal, OBSTACLES, testing)

    grrt.solve(eps, it)   # Solve the Geometric RRT problem with eps precision and it iterations

####################


if __name__ == "__main__":
    RRTsolver()