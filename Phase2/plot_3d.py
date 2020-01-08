from __future__ import division
import math
import numpy as np
from scipy.integrate import odeint
from trajectory_planner import TrajectoryGenerator

G=9.8 #m/s^2
M = 0.030 #kg
KF=6.11*1e-8 #N/rpm^2
KM=1.5*1e-9 #N*m/rpm^2

#attitude controls gains
kdroll=25
kproll=10
kdpitch=25
kppitch=10
kdyaw=25
kpyaw=10

#altitude  control gains
kdx=5
kpx=4
kdz=5
kpz=4
kdy=5
kpy=4

I_moment= [[1.43*1e-5,0,0],[0,1.43*1e-5,0,0],[0,0,2.89*1e-5]]

def pos_control3D(input_vector,feedback):
        u1=M*(G+input_vector[8]+(kdz*(input_vector[5]-feedback[5])+kpz*(input_vector[2]-feedback[2]))) #altitude(thrust)
        roll=(-1/G)*((input_vector[7]+kdy*(input_vector[4]-feedback[4]))+kpy*(input_vector[1]-feedback[1])) #y
        pitch=(-1/G)*((input_vector[6]+kdx*(input_vector[3]-feedback[3]))+kpx*(input_vector[0]-feedback[0]))#x
        return u1,roll,pitch

def att_control3D(roll,roll_vel,pitch,pitch_vel,yaw,yaw_vel,feedback):
       # print(roll,roll_vel,pitch,pitch_vel,yaw,yaw_vel)
        u2_=np.zeros(3)
        u2_[0]= I_moment[0][0]*(kdroll*(roll_vel-feedback[1])+kproll*(roll-feedback[0]))
        u2_[1]= I_moment[1][1]*(kdpitch*(pitch_vel-feedback[3])+kppitch*(pitch-feedback[2]))
        u2_[2]= I_moment[2][2]*(kdyaw*(yaw_vel-feedback[5])+kpyaw*(yaw-feedback[4]))
        #print(u2_)
        return u2_

def model_state(y,t,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,u1,u2):

       a=   [ x2, # x'
              -G* (x9), #x''
              x4, #y' = x3
              -G*(x7), #y''
              x6, #z'
              -G+(u1/M), #z''
              x8, #roll'
              u2[0]/I_moment[0][0],#roll''
              x10, #pitch'
              u2[1]/I_moment[1][1], #pitch''
              x12, #yaw'
              u2[2]/I_moment[2][2]#yaw'']
              ]
       return a

def path3D(waypoints,norm,takeout,landing):
        T = 1
        dt = T/100
        u2=[0,0,0]
        state=[waypoints[0][0],0,waypoints[0][1],0,waypoints[0][2],0,0,0,0,0,0,0]
        max_vel = 2/norm #normalize the speed of the copter
        max_acc = 1/norm
        poly =[]
        yaw = 0
        feedback_attitude = [0,0,0,0,0,0]  # angle and angle velocity
        feedback_position = [waypoints[0][0],waypoints[0][1],waypoints[0][2],0,0,0]
        for i in range((len(waypoints)-1)):
            elapsed = 0
            traj = TrajectoryGenerator(waypoints[i], waypoints[i+1], T)
            traj.solve()
            x_coeffs = traj.x_c
            y_coeffs = traj.y_c
            z_coeffs = traj.z_c

            dist = math.sqrt((waypoints[i][0] - waypoints[i+1][0]) ** 2 + (waypoints[i][1] - waypoints[i+1][1]) ** 2 + (waypoints[i][2] - waypoints[i+1][2])**2)
            tolerance = dist * 0.10
            # using the norm 2 distance to see if quadrotor is close to destination

            while(elapsed < T):
                #---------desired input vector-----------------------#
                # = time.time() - start  # check time to see if meets desired hovering time

                if  takeout == False and landing == False:
                    des_x_pos = calculate_position(x_coeffs, elapsed)[0]
                    des_y_pos = calculate_position(y_coeffs, elapsed)[0]
                    des_z_pos = calculate_position(z_coeffs, elapsed)[0]
                    des_x_vel = calculate_velocity(x_coeffs, elapsed)[0]
                    des_y_vel = calculate_velocity(y_coeffs, elapsed)[0]
                    des_z_vel = calculate_velocity(z_coeffs, elapsed)[0]

                    des_x_acc = calculate_acceleration(x_coeffs, elapsed)[0]
                    des_y_acc = calculate_acceleration(y_coeffs, elapsed)[0]
                    des_z_acc = calculate_acceleration(z_coeffs, elapsed)[0]
                elif takeout == True and landing == False:
                    des_x_pos = 0
                    des_y_pos = 0
                    des_z_pos = calculate_position(z_coeffs, elapsed)[0]
                    des_x_vel = 0
                    des_y_vel = 0
                    des_z_vel = max_vel
                    des_x_acc = 0
                    des_y_acc = 0
                    des_z_acc = max_acc
                elif takeout == False and landing == True:
                    des_x_pos = calculate_position(x_coeffs, elapsed)[0]
                    des_y_pos = calculate_position(y_coeffs, elapsed)[0]
                    des_z_pos = calculate_position(z_coeffs, elapsed)[0]
                    des_x_vel = 0
                    des_y_vel = 0
                    des_z_vel = -max_vel
                    des_x_acc = 0
                    des_y_acc = 0
                    des_z_acc = -max_acc


                if (abs(des_x_vel)> max_vel):
                    des_x_vel = np.sign(des_x_vel)*max_vel
                if (abs(des_y_vel) > max_vel):
                    des_y_vel = np.sign(des_y_vel)*max_vel
                if (abs(des_z_vel)> max_vel):
                    des_z_vel = np.sign(des_z_vel)*max_vel

                if (abs(des_x_acc) > max_acc):
                    des_x_acc = np.sign(des_x_acc)*max_acc
                if (abs(des_y_acc)> max_acc):
                    des_y_acc = np.sign(des_y_acc)*max_acc
                if (abs(des_z_acc)> max_acc):
                    des_z_acc = np.sign(des_z_acc)*max_acc

                a=[des_x_pos, des_y_pos, des_z_pos, #desired position x,y,z
                   des_x_vel,  #x-vel
                   des_y_vel, #y-vel
                   des_z_vel,#z-vel

                   des_x_acc,#acc-x
                   des_y_acc,#acc-y
                   des_z_acc]#acc-z
                [u1,roll,pitch]= pos_control3D(a, feedback_position)

                roll_vel = math.cos(pitch)*state[7]-math.cos(roll)*math.sin(pitch)*state[11]
                pitch_vel = state[9]+math.sin(roll)*state[11]
                yaw_vel = math.sin(pitch)*state[9]+math.cos(roll)*math.cos(pitch)*state[11]
                #yaw += yaw_vel * dt

                u2[0], u2[1], u2[2] = att_control3D(roll, roll_vel, pitch,pitch_vel, yaw, yaw_vel, feedback_attitude)

                #***feedback****#

                x1=state[0] #x
                x2=state[1] #x'
                x3=state[2] #y
                x4=state[3] #y'
                x5=state[4] #z
                x6=state[5] #z'
                x7=roll     #roll
                x8=roll_vel #roll'
                x9=pitch    #pitch
                x10=pitch_vel#pitch'
                x11=yaw #yaw
                x12=yaw_vel      #yaw'
                state= odeint(model_state,state,[elapsed,elapsed+dt],args=(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,u1,u2))[1]

                feedback_attitude=[state[6], state[7], state[8], state[9], state[10], state[11]] #angle and angle velocity

                #constaint  max velocity = 2m/s
                if (abs(state[1]) > max_vel):
                    state[1] = np.sign(state[1]) * max_vel
                if (abs(state[3]) > max_vel):
                    state[3]= np.sign(state[3]) * max_vel
                if (abs(state[5]) > max_vel):
                    state[5] = np.sign(state[5]) * max_vel

                actual_x=state[0]
                actual_xvel=state[1]
                actual_y=state[2]
                actual_yvel=state[3]
                actual_z=state[4]
                actual_zvel=state[5]

                dist = math.sqrt((waypoints[i+1][0] - actual_x) ** 2 + (waypoints[i+1][1] - actual_y) ** 2 + (waypoints[i+1][2] - actual_z) ** 2)
                if  dist < tolerance:
                    break
                poly_eq = [actual_x, actual_y, actual_z]
                elapsed = elapsed + dt  # check time to see if meets desired hovering time

                poly.append(poly_eq)
                #des.append(des_eq)
                feedback_position=[actual_x,actual_y,actual_z,actual_xvel,actual_yvel,actual_zvel]  #feed_back x,y, w/ their vels

        eq = [waypoints[len(waypoints)-1][0], waypoints[len(waypoints)-1][1], waypoints[len(waypoints)-1][2]]
        poly.append(eq)
        return poly


#Calculates a position given a set of quintic coefficients and a time.
def calculate_position(c, t):
    return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]

#Calculates a velocity given a set of quintic coefficients and a time.
def calculate_velocity(c, t):

    return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]

#   Calculates an acceleration given a set of quintic coefficients and a time.
def calculate_acceleration(c, t):
    return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]

