from __future__ import division
from time import sleep
import math 
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
G=9.8 #m/s^2
M = 0.030 #kg
KF=6.11*1e-8 #N/rpm^2
KM=1.5*1e-9 #N*m/rpm^2

#attitude controls gains
kdroll=20
kproll=150
kdr=20
kppitch=150
kdpitch=20
kpyaw=150
kdyaw=20

#altitude  control gains
kdy=10
kpy=20
kdz=3
kpz=5
kdy=1
kpy=2
speed=10
I_moment= [[1.43*1e-5,0,0],[0,1.43*1e-5,0,0],[0,0,2.89*1e-5]]
tolerance=4.5
   


'''
        @param: input_vector:(desired position values) 
[x,y,z,x_vel,y_vel,z_vel,x_acc,y_acc,z_acc]   
                feedback:(actual position values)                   [x,y,z,x_vel,y_vel,z_vel]

'''

def pos_control3D(input_vector,feedback):                
              #TODO: Check equation

        u1=M*(G+input_vector[8]+kdz*(input_vector[5]-feedback[5])+kpz*(input_vector[2]-feedback[2]))
        roll=(-1/G)*((input_vector[7]+kdy*(input_vector[4]-feedback[4]))+kpy*(input_vector[1]-feedback[1])) #y
        pitch=(-1/G)*((input_vector[6]+kd*(input_vector[3]-feedback[3]))+kpy*(input_vector[0]-feedback[0]))#x
        yaw=(-1/G)*((input_vector[8]+kd*(input_vector[5]-feedback[2]))+kpy*(input_vector[2]-feedback[2]))#z
  
       
        return u1,roll,pitch,yaw        

def att_control3D(phi,theta,yaw,feedback): 
                        
        u2[0]=(I_moment[0][0]*((0+ kdroll*(0-feedback[1])))+kproll*(phi-feedback[0]))
        u2[1]=(I_moment[1][1]*((0+ kdpitch*(0-feedback[1])))+kppitch*(theta-feedback[0]))
        u2[3]=(I_moment[2][2]*((0+ kdyaw*(0-feedback[1])))+kpyaw*(yaw-feedback[0]))                   
        return u2

def model_z2(y,t,u1): 
        theta,omega=y 
        dzdu2=[omega,-G+(u1/M)]
        return dzdu2

def model_y_phi2(y,t,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,u1,u2):
       return x2,G*x7,x4,-G*x9,x6,-G+(u1/M),x8,u2/I_moment[0][0],x10, u2/I_moment[1][1],x12, u2/I_moment[2][2]

                
def path3D(q0,q1,dimensionality):
        dt=0    
        u1=0 #thrush           
        map_position_z =[]
        map_position_y =[]
        map_position_x =[]
        state_new=[q0[0],0,q0[1],0,q0[2],0,0,0,0,0,0,0]        
        z=[q0[1],0]
        u2=[0,0,0]
        feedback_attitude=[0,0,0,0]       
        feedback_position=[q0[0],q0[1],q0[2],0,0,0]
        actual_z=0
        actual_y=0
        actual_x=0

        diff_phi=[q1[0]-q0[1],q1[0]-q0[1]]    
        if(diff_phi[0]==0):
                phi_desired = math.pi/2

        else:        
                d=diff_phi[1]/diff_phi[0]               
                phi_desired=math.atan(d)

        diff_pitch=[q1[0]-q0[2],q1[0]-q0[2]]   
        if(diff_pitch[0]==0):
                pitch_desired = math.pi/2 
        else:        
                d=diff_pitch[1]/diff_pitch[0]               
                pitch_desired=math.atan(d)
        
        diff_yaw=[q1[1]-q0[2],q1[1]-q0[2]]    
        if(diff_yaw[1]==0):
                yaw_desired = math.pi/2

        else:        
                d=diff_yaw[1]/diff_yaw[0]               
                yaw_desired=math.atan(d)

        F = (10)/(math.sin(phi_desired)+math.sin(pitch_desired)) #choose value greater than >0 to a pose gravity         
        state=[q0[0],0,0,0]
        inrange=False
        dist = math.sqrt((q1[0] - q0[0])**2+(q1[1] - q0[1])**2+(q1[1] - q0[1])**2) #using the norm 2 distance to see if quadrotor is close to destination
        while(dist > tolerance or inrange):
               
               a=[q1[0],q1[1],q1[2],
               F*dt*(math.sin(pitch_desired)+math.sin(phi_desired)),
               F*dt*(math.sin(phi_desired)+math.cos(pitch_desired)),
               F*dt*(math.sin(pitch_desired)- math.cos(pitch_desired)),
               F*dt*(-math.cos(phi_desired)-math.cos(yaw_desired)),
               F*dt*(math.cos(phi_desired)+math.cos(yaw_desired)),
               F*dt*(math.sin(pitch_desired)+math.sin(phi_desired))]#des,vel,acc                 
                dt=dt+0.00001

                [u1,phi0,theta,yaw]= pos_control3D(a,feedback_position)  
              
                for i in range(speed):
                 #TODO: check if z should be inside the loop or not???
                  z = odeint(model_z2,z,[0,dt],args=(u1,))[1] #(position-z,  velocity-z)               

                        u2[0:2] = att_control3D(phi0,theta,yaw,feedback_attitude)
                        dt=dt+0.001
                
                        #***feedback****#
 
                        x1=state[0]
                        x2=state[1]
                        x3=state[2]      
                        x4=state[3]
                        x5=state[4]                                                
                        x6=state[5]
                        x7=state[6]
                        x8=state[7]
                        x9=state[8]
                        x10=state[9]
                        x11=state[10]
                        x12=state[11]
                        state= odeint(model_y_phi2,state_new,[0,dt],args=(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10, x11,u1,u2))[1]


                        state_new= [state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7],state[8],state[9],state[10],state[11]]                      
                        
                        feedback_attitude=[state[6],state[7],state[8],state[9],state[10],state[11]]#angle and angle velocity for r,p
        
                        actual_z= z[0]
                        actual_z_velocity
                        actual_y=state[0]                       
                        if (dimensionality==2):
                                actual_x=state[0]                               
                        map_position_y.append(state[0])
                        map_position_z.append(z[0])
                        map_position_x.append(z[0])
                        dist = math.sqrt((q1[0] - q0[0])**2+(q1[1] - q0[1])**2+(q1[1] - q0[1])**2) #using the norm 2
                        if (dist < tolerance):            
                             inrange= True  
                             if(old_dist < dist): #validate if target can become more accruate 
                                return [map_position_x,map_position_y,map_position_z]
                        old_dist= dist                                                          
                        
                feedback_position=[actual_x,actual_y,actual_z,actual_xvel,actual_yvel,actual_zvel]  #feed_back x,y, w/ their vels
                print("position")          
                print(feedback_position)
 
        return [map_position_x,map_position_y,map_position_z]
                        

       
