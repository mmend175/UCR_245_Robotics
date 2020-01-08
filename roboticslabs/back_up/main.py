from __future__ import division
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import matplotlib.pyplot as plt
from path import motion
import scipy
from trajectory_planner import TrajectoryGenerator
from move_quadrotor import move_quadrotor
from plot_3d import path3D
from plot_3d_d import path3D_d

def phase2_main():
    # Initialize Map Size
    map_size = [0, 0, 0, 20, 20, 20]
    '''map_size[0] = int(input("Choose x origin for your map: "))
    map_size[1] = int(input("Choose y origin for your map: "))
    map_size[2] = int(input("Choose z origin for your map: "))
    map_size[3] = int(input("Choose width of your map: "))
    map_size[4] = int(input("Choose length of your map: "))
    map_size[5] = int(input("Choose height of your map: ")) '''

    # Initialize Map Obstacles
    num_obs = 2  # int(input("Choose how many obstacles there will be: "))
    obstacles = [[0 for j in range(6)] for k in range(num_obs)]
    '''
    for i in range(num_obs):
        obs_array = [20, 20, 20, 20, 20, 20]
        obs_array = [20, 20, 20, 20, 20, 20]
        
        obs_array[0] = int(input("Choose x origin for obstacle " + str(i+1) + ":"))
        obs_array[1] = int(input("Choose y origin for obstacle " + str(i+1) + ":"))
        obs_array[2] = int(input("Choose z origin for obstacle " + str(i+1) + ":"))
        obs_array[3] = int(input("Choose width for obstacle " + str(i+1) + ":"))
        obs_array[4] = int(input("Choose length origin for obstacle " + str(i+1) + ":"))
        obs_array[5] = int(input("Choose height origin for obstacle " + str(i+1) + ":"))
        
        obstacles[i] = obs_array
    '''
    obstacles[0] = [2 ,2 , 0, 10, 10, 10]
    obstacles[1] = [10, 10, 10, 5, 5, 5]
    # Map limits
    max_x = map_size[3] - map_size[0]
    max_y = map_size[4] - map_size[1]
    max_z = map_size[5] - map_size[2]

    # Initialize q0
    q0 = [0, 0, 0]
    '''q0_x = int(input("Choose initial x position for your quadrotor: "))
    while (q0_x > max_x) or (q0_x < map_size[0]):
        print("INVALID x pos, try again...")
        q0_x = int(input("Choose initial x position for your quadrotor: "))
    q0[0] = q0_x
    q0_y = int(input("Choose initial y position for your quadrotor: "))
    while (q0_y > max_y) or (q0_y < map_size[1]):
        print("INVALID y pos, try again...")
        q0_y = int(input("Choose initial y position for your quadrotor: "))
    q0[1] = q0_y
    q0_z = int(input("Choose initial z position for your quadrotor: "))
    while (q0_z > max_z) or (q0_z < map_size[2]):
        print("INVALID z pos, try again...")
        q0_z = int(input("Choose initial z position for your quadrotor: "))
    q0[2] = q0_z'''

    # Initialize qh
    qh = [max_x, max_y, max_z]
    '''qh_x = int(input("Choose final x position for your quadrotor: "))
    while (qh_x > max_x) or (qh_x < map_size[0]):
        print("INVALID x pos, try again...")
        qh_x = int(input("Choose final x position for your quadrotor: "))
    qh[0] = qh_x
    qh_y = int(input("Choose final y position for your quadrotor: "))
    while (qh_y > max_y) or (qh_y < map_size[1]):
        print("INVALID y pos, try again...")
        qh_y = int(input("Choose final y position for your quadrotor: "))
    qh[1] = qh_y
    qh_z = int(input("Choose final z position for your quadrotor: "))
    while (qh_z > max_z) or (qh_z < map_size[2]):
        print("INVALID z pos, try again...")
        qh_z = int(input("Choose final z position for your quadrotor: "))
    qh[2] = qh_z'''

    print("Map Size (x,y,z,w,l,h):")
    print(map_size)
    print("obstacles (x,y,z,w,l,h):")
    print(obstacles)
    print("Initial Position (q0):")
    print(q0)
    print("Final Position (qh)")
    print(qh)

    # config_space = configure(map_size, obstacles)
    waypoints = motion(q0, qh, map_size, obstacles)
    print("leng")
    print(len(waypoints))
    waypoints =np.array(waypoints)




    #waypoints = [[0, 0, 0], [0, 5, 0], [0, 5, 5], [5, 5, 5], [10, 10, 10]]
    poly=[]

    T = 1

    # BEGIN PLOTTING
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    poly,des = path3D(waypoints, T)

    #poly = [map_position_x,map_position_y,map_position_z]
    print(poly)
    #poly = move_quadrotor(x_coeffs, y_coeffs, z_coeffs, T)
    #print(x_coeffs, y_coeffs, z_coeffs)

    # Plotting Map Space
    mx = [map_size[0], map_size[0] + map_size[3]]
    my = [map_size[1], map_size[1] + map_size[4]]
    mz = [map_size[2], map_size[2] + map_size[5]]
    drawRect(ax, mx, my, mz, 'cyan', 'b', 0.08)

    # Plotting Obstacles
    for i in range(num_obs):
        ob_x = [obstacles[i][0], obstacles[i][3]+obstacles[i][0]]
        ob_y = [obstacles[i][1], obstacles[i][4]+obstacles[i][1]]
        ob_z = [obstacles[i][2], obstacles[i][5]+obstacles[i][2]]
        drawRect(ax, ob_x, ob_y, ob_z, 'maroon', 'b', 0.5)

    # Plotting Chosen Path
    # TODO loop through and draw algorithm's chosen points, then plot path between points
    # ax.scatter3D(q0[0], q0[1], q0[2], c='g')
    # ax.scatter3D(qh[0], qh[1], qh[2], c='y')

    wx = np.asanyarray([0 for i in range(len(waypoints))],dtype=float)
    wy = np.asanyarray([0 for i in range(len(waypoints))],dtype=float)
    wz = np.asanyarray([0 for i in range(len(waypoints))],dtype=float)
    cx = np.asanyarray([0 for i in range(len(poly))],dtype=float)
    cy = np.asanyarray([0 for i in range(len(poly))],dtype=float)
    cz = np.asanyarray([0 for i in range(len(poly))],dtype=float)
    dx = np.asanyarray([0 for i in range(len(des))],dtype=float)
    dy = np.asanyarray([0 for i in range(len(des))],dtype=float)
    dz = np.asanyarray([0 for i in range(len(des))],dtype=float)

    for i in range(len(waypoints)):
        wx[i] = waypoints[i][0]
        wy[i] = waypoints[i][1]
        wz[i] = waypoints[i][2]
        #ax.scatter3D(wx[i], wy[i], wz[i], c='b')
    ax.plot3D(wx, wy, wz, 'red',label='waypoints lines')
    for i in range(len(poly)):
        cx[i] = poly[i][0]
        cy[i] = poly[i][1]
        cz[i] = poly[i][2]
        dx[i] = des[i][0]
        dy[i] = des[i][1]
        dz[i] = des[i][2]
        #ax.scatter3D(cx[i], cy[i], cz[i], c='y')
        #ax.scatter3D(dx[i], dy[i], dz[i], c='y')
    ax.plot3D(cx, cy, cz, 'green',label='quadrotor path')
    #ax.plot3D(dx, dy, dz, c='b')

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
    ax.add_collection3d(Poly3DCollection(vertices, facecolors=Fcolor, linewidths=1, edgecolors=Ecolor, alpha=a))


if __name__ == '__main__':
    phase2_main()

