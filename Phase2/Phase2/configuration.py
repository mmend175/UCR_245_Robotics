import numpy as np
import time

# robot dimensions
# .092X0.092X0.029 (m)
# 92mm x 92mm x 29mm
norm_size = 5
robot_width = 3*norm_size
robot_length = 3*norm_size
robot_height = 1*norm_size


# function will create the environment for robot as a binary grid. free space = 0,  obstacles = 1.
def configure(map_size,  obstacles):
    environment = np.zeros((map_size[3],  map_size[4],  map_size[5]))
    # need to start from each obstacle and check through environment
    t0 = time.time()
    for m in range(len(obstacles)):
        print("Mapping C_obs for obstacle " + str(m+1) + "...")
        for i in range(obstacles[m][0] - robot_width,  obstacles[m][0] + obstacles[m][3] + robot_width):  # width of map
            if i >= map_size[3] or i < map_size[0]:
                continue
            else:
                for j in range(obstacles[m][1] - robot_length,  obstacles[m][1] + obstacles[m][4] + robot_length):  # length of map
                    if j >= map_size[4] or j < map_size[1]:
                        continue
                    else:
                        for k in range(obstacles[m][2] - robot_height,  obstacles[m][2] + obstacles[m][5] + robot_height):  # height of map
                            if k >= map_size[5] or k < map_size[2]:
                                continue
                            else:
                                environment[i,  j,  k] = 1
    t1 = time.time()
    totalTime = t1-t0

    return environment, totalTime
