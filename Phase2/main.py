from __future__ import division

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import matplotlib.pyplot as plt
# from Phase2.path import motion
from trajectory_planner import TrajectoryGenerator
from readInputFile import readInputFile
from astar_planner import motion_planner
from configuration import configure
from plot_3d import path3D
from manual_input import user_input
import math
import time
from path import motion

def phase2_main():
    q0,  qh,  map_size,  obstacles,  num_obs = [],  [],  [],  [],  0
    manual_input = int(input("Would you like to use an input file? (Choose 0) or manual input mode? (Choose 1): "))
    while manual_input != 0 and manual_input != 1:
        print("Invalid input,  please try again...")
        manual_input = int(input("Would you like to use an input file? (Choose 0) or manual input mode? (Choose 1): "))

    if not manual_input:
        filepath = input("Please enter your file name ('sample.txt'): ")
        map_sz,  obstacles = readInputFile(filepath)
        q0 = [0,  0,  0]
        qh = [map_sz[0],  map_sz[1],  map_sz[2]]
        map_size = [q0[0],  q0[1],  q0[2],  qh[0],  qh[1],  qh[2]]
        num_obs = len(obstacles)
    elif manual_input:
        map_size, obstacles, q0, qh = user_input()
        num_obs = len(obstacles)
    # takeoff_h = math.ceil(map_size[2]/4)

    algorithm = int(input("Would you like to use an A-star algorithm? (Choose 0) or Rapidly Exploring Random Tree Star(aka RRT*) ? (Choose 1): "))
    tStart = time.time()

    print("Map Size (x, y, z, w, l, h):")
    print(map_size)
    print("obstacles (x, y, z, w, l, h):")
    print(obstacles)
    print("Initial Position (q0):")
    print(q0)
    print("Final Position (qh)")
    print(qh)

    # Normalizing for the quadrotor size ------------------------------------------------------------------------------
    norm_quad = 0.029
    for i in range(len(map_size)):
        map_size[i] = math.ceil(map_size[i] / norm_quad)  # normalize for robot size
    for i in range(len(obstacles)):
        for j in range(len(obstacles[i])):
            obstacles[i][j] = math.ceil(obstacles[i][j] / norm_quad)  # normalize for robot size
    qh[0], qh[1], qh[2] = math.ceil(qh[0] / norm_quad), math.ceil(qh[1] / norm_quad), math.ceil(qh[2] / norm_quad)

    takeoff = []
    takeoff.append([0,0,0])

    q0 = [q0[0], q0[1], q0[0] +int(map_size[5]/5)]
    takeoff.append(q0)

        # -----------------------------------------------------------------------------------------------------------------
    if(algorithm==0):
        # Solve for CSpace and find a feasible path -----------------------------------------------------------------------
        config_space, cTime = configure(map_size,  obstacles)
        print("CONFIG DONE")
        print("Mapping A-Star...")
        unsamp_waypoints, astar_time = motion_planner(q0,  qh,  config_space)    # A* Algorithm
        # waypoints = motion(q0,  qh,  map_size,  obstacles)   # Probability Tree
        print("length")
        print(len(unsamp_waypoints))

        sample_rate = 10    # Sample waypoints to smooth path
        way_len = math.ceil(len(unsamp_waypoints) / sample_rate)
        waypoints = [[0, 0, 0] for i in range(way_len)]
        count = 0
        for i in range(len(unsamp_waypoints)):
            if i % sample_rate == 0:
                waypoints[count] = unsamp_waypoints[i]
                count += 1
        waypoints.append([qh[0], qh[1], qh[2]])

    else:
        print("Mapping RRT-Star...")
        rrt_time, cTime, waypoints = motion(q0, qh, map_size, obstacles)
        waypoints = waypoints.tolist()


    # -----------------------------------------------------------------------------------------------------------------

    poly = path3D(waypoints,norm_quad,False,False)
    tFinal = time.time()
    totalTime = tFinal - tStart
    land=[]
    land.append(waypoints[len(waypoints)-1])#land copter at the end of path
    land.append([waypoints[len(waypoints)-1][0],waypoints[len(waypoints)-1][1],0])

    takeoff_points= path3D(takeoff, norm_quad,True,False)
    landed = path3D(land,norm_quad,False,True)
    poly_takeoff = takeoff_points.copy()

    takeoff_points.extend(waypoints)
    waypoints= takeoff_points.copy()
    waypoints.extend(landed)

    poly_takeoff.extend(poly)
    poly= poly_takeoff.copy()
    poly.extend(landed)


    # Times for each portion of code
    if (algorithm == 0):
        print("C_space Time: %.4f" % cTime)
        print("Algorithm Time: %.4f" % astar_time)

    else:
        print("C_space Time: %.4f" % cTime)
        print("Algorithm Time: %.4f" % rrt_time)
    print("Program Time: %.4f" % totalTime)

    # BEGIN PLOTTING --------------------------------------------------------------------------------------------------

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    mx = [map_size[0], map_size[0] + map_size[3]]
    my = [map_size[1], map_size[1] + map_size[4]]
    mz = [map_size[2], map_size[2] + map_size[5]]
    drawRect(ax, mx, my, mz, 'cyan', 'b', 0.08)

    # Plotting Obstacles
    for i in range(num_obs):
        ob_x = [obstacles[i][0], obstacles[i][3] + obstacles[i][0]]
        ob_y = [obstacles[i][1], obstacles[i][4] + obstacles[i][1]]
        ob_z = [obstacles[i][2], obstacles[i][5] + obstacles[i][2]]
        drawRect(ax, ob_x, ob_y, ob_z, 'maroon', 'r', 0.5)

    # Plotting Chosen Path
    wx = np.asanyarray([0 for i in range(len(waypoints))], dtype=float)
    wy = np.asanyarray([0 for i in range(len(waypoints))], dtype=float)
    wz = np.asanyarray([0 for i in range(len(waypoints))], dtype=float)
    cx = np.asanyarray([0 for i in range(len(poly))], dtype=float)
    cy = np.asanyarray([0 for i in range(len(poly))], dtype=float)
    cz = np.asanyarray([0 for i in range(len(poly))], dtype=float)
    for i in range(len(waypoints)):
        wx[i] = waypoints[i][0]
        wy[i] = waypoints[i][1]
        wz[i] = waypoints[i][2]
        # ax.scatter3D(wx[i], wy[i], wz[i], c='b')
    ax.plot3D(wx, wy, wz, 'black', label='waypoints ')
    for i in range(len(poly)):
        cx[i] = poly[i][0]
        cy[i] = poly[i][1]
        cz[i] = poly[i][2]
    ax.plot3D(cx, cy, cz, 'green', label='quadrotor path')
    #ax.plot3D(dx, dy, dz, c='black',label='desired')

    ax.legend()

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Show Final Plot
    plt.show()


def drawRect(ax, X, Y, Z, Fcolor, Ecolor, a):
    # Labels the point values of each vertex of the rectangle
    v = np.array([[X[0], Y[0], Z[0]], [X[1], Y[0], Z[0]],
                  [X[1], Y[1], Z[0]], [X[0], Y[1], Z[0]],
                  [X[0], Y[0], Z[1]], [X[1], Y[0], Z[1]],
                  [X[1], Y[1], Z[1]], [X[0], Y[1], Z[1]]])
    ax.scatter3D(v[:, 0], v[:, 1], v[:, 2])

    # Groups Vertices according to each surface of the rectangle
    vertices = [[v[0], v[1], v[2], v[3]], [v[0], v[1], v[5], v[4]],
                [v[1], v[2], v[6], v[5]], [v[2], v[3], v[7], v[6]],
                [v[3], v[0], v[4], v[7]], [v[4], v[5], v[6], v[7]]]

    # Add Rectangle to the figure
    rectangle= Poly3DCollection(vertices, facecolors=Fcolor, linewidths=1, edgecolors=Ecolor, alpha=a)
    rectangle.set_facecolor(Fcolor)
    ax.add_collection3d(rectangle)

if __name__ == '__main__':
    phase2_main()
