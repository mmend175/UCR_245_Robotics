import numpy as np

from randomtree import Randomtree
from configure_space import ConfigureSpace
from rrt_star import RRTStar
from scipy import interpolate
import time
norm_size = 5
robot_width = 3*norm_size
robot_length = 3*norm_size
robot_height = 1*norm_size
#robot_width = 0.092
#robot_length =0.092
#robot_height =0.092/3
robot_width_pad = robot_width*2
robot_length_pad =robot_length*2
robot_height_pad =robot_width *2

def motion(init_pos,goal_pos,map_size,obstacle):

        x = init_pos[0]
        y = init_pos[1]
        z = init_pos[2]
        obstacle= np.array(obstacle)
        obstacle_list_aug = np.zeros((obstacle.shape[0],obstacle.shape[1]))
        obstacle_list_aug_pad= np.zeros((obstacle.shape[0], obstacle.shape[1]))
        for i in range (len(obstacle)):
                obstacle_list_aug[i][0] = max(obstacle[i][0]-robot_width,0)
                obstacle_list_aug[i][1] = max(obstacle[i][1]-robot_length,0)
                obstacle_list_aug[i][2] = max(obstacle[i][2]-robot_height,0)
                obstacle_list_aug[i][3] = max(obstacle[i][0]+obstacle[i][3]+robot_width,0)
                obstacle_list_aug[i][4] = max(obstacle[i][1]+obstacle[i][4]+robot_length,0)
                obstacle_list_aug[i][5] = max(obstacle[i][2]+obstacle[i][5]+robot_height,0)
        for i in range(len(obstacle)):
                obstacle_list_aug_pad[i][0] = max(obstacle[i][0] - robot_width_pad, 0)
                obstacle_list_aug_pad[i][1] = max(obstacle[i][1] - robot_length_pad, 0)
                obstacle_list_aug_pad[i][2] = max(obstacle[i][2] - robot_height_pad, 0)
                obstacle_list_aug_pad[i][3] = max(obstacle[i][0] + obstacle[i][3] + robot_width_pad, 0)
                obstacle_list_aug_pad[i][4] = max(obstacle[i][1] + obstacle[i][4] + robot_length_pad, 0)
                obstacle_list_aug_pad[i][5] = max(obstacle[i][2] + obstacle[i][5] + robot_height_pad, 0)

        X_dimensions = np.array([(map_size[0], map_size[3]), (map_size[1], map_size[4]), (map_size[2], map_size[5])])  # dimensions of Search Space
# obstacles
        x_init = (x, y, z)  # starting location
        x_goal = (goal_pos[0], goal_pos[1], goal_pos[2])  # goal location

        Q = np.array([(norm_size*8, 4)])  # length of tree edges
        r =norm_size/0.1# length of smallest edge to check for intersection with obstacles
        max_samples = 1024*norm_size*3 # max number of samples to take before timing out
        prc = 0.1 # probability of checking for a connection to goal
        rewire_count = 8# optional, number of nearby branches to rewire

# Configure Space
        t0 = time.time()
        grid = ConfigureSpace(X_dimensions, obstacle_list_aug_pad)
        t1 = time.time()
        c_time = t1 - t0

        grid1 = ConfigureSpace(X_dimensions, obstacle_list_aug)

# create rrt_search
        #rrt = Randomtree(grid, Q, x_init, x_goal, max_samples, r, prc)
       # path = rrt.rrt_search()
        t2 = time.time()
        rrt = RRTStar(grid, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path = rrt.rrt_star(norm_size)
        t3 = time.time()
        rr_time = t3 - t2
        path = np.array(path)
        if (len(path) > 3):
                tck, u = interpolate.splprep(path.T)
                new = interpolate.splev(np.linspace(0, 1, len(path) * 2), tck)
                new = np.transpose(new)

                new_x = np.clip(new[:, 0], a_min=map_size[0], a_max=map_size[0] + map_size[3]-robot_width)[
                        :]  # check map limit in x direction
                new_y = np.clip(new[:, 1], a_min=map_size[1], a_max=map_size[0] + map_size[4]-robot_length)[
                        :]  # check map limit in y direction
                new_z = np.clip(new[:, 2], a_min=map_size[2], a_max=map_size[0] + map_size[5]-robot_height)[
                        :]  # check map limit in z direction
                new_x = new_x[:, np.newaxis]
                new_y = new_y[:, np.newaxis]
                new_z = new_z[:, np.newaxis]
                new = np.hstack((new_x, new_y, new_z))
        else:
                new = path

        for x in range(len(new)):
                if (not grid1.obstacle_free(tuple(new[x]))):
                        new = path
                        #time.sleep(1000)
                        break

        return c_time, rr_time, new