from __future__ import division
from time import sleep
import math 
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import time
G=-9.8 #m/s^2
M = 0.030 #kg
KF=6.11*1e-8 #N/rpm^2
KM=1.5*1e-9 #N*m/rpm^2

#attitude controls gains
kdrow=0
kprow=0

#altitude  control gains
kdy=4
kpy=5
kdz=4
kpz=5
speed=10
I_moment= [[1.43*1e-5,0,0],[0,1.43*1e-5,0,0],[0,0,2.89*1e-5]]
tolerance=0
        
def pos_control(input_vector,feedback):                
        
        u1=M*(G+input_vector[5]+kdz*(input_vector[3]-feedback[3])+kpz*(input_vector[1]-feedback[1]))
       
        phi=(-1/G)*((input_vector[4]+kdy*(input_vector[2]-feedback[2]))+kpy*(input_vector[0]-feedback[0]))
  
        return u1, phi;        

def att_control2D(phi,feedback):
                        
        u2_x=I_moment[0][0]*((0+ kdrow*(0-feedback[1]))+kprow*(phi-feedback[0]))
        return u2_x

def model_z2(y,t,u1): 
        theta,omega=y 
        dzdu2=[omega,-G+(u1/M)]
        return dzdu2

def model_y_phi2(y,t,x1,x2,x3,x4,x5,x6, u1,u2):
       return x2,-G*x5,x4,-G+(u1/M),x6, u2/I_moment[0][0]
                
def path2D(q0,q1,T):
        dt=0    
        u1=0
        map_position_z =[]
        map_position_y =[]
        epoch_list=[] #used for house keeping 
        state=[q0[0],0,q0[1],0,0,0]        
        z=[q0[1],0]
        u2=0
        feedback_attitude=[0,0]       
        feedback_position=[q0[0],q0[1],0,0]
        actual_z=q0[1]
        actual_y=q0[0]

        diff=[q1[1]-q0[1],q1[0]-q0[0]]       
        i = 0;      
        if(diff[0]==0):
                theta = 0
        else:        
                d=diff[1]/diff[0]
                theta=math.atan(d)
       
        F = 0.1
        inrange=False
        dist = math.hypot(q1[0] - q0[0], q1[1] - q0[1]) #using the norm 2 distance to see if quadrotor is close to destination
        tolerance = dist*0.05 #impliment such that the distance is 10 percent from start to end 
        loopCond = True
        hover = False

        if(q0==q1):        
               hover = True                       #set hover to start
               start = time.time()                      
               elapsed=0
               time.clock()  
        
        while(loopCond):
                if(q0==q1 and elapsed < T):        # If both points are equal, path is in hover mode
                        loopCond = True            # Hover until T hover time is reached
                elif(dist > tolerance or inrange): # Standard Loop Condition
                        loopCond = True
                else:
                        loopCond = False
                if (diff[0] < 0):
                        direction = -1
                else:
                        direction = 1
                a=[q1[0],q1[1],F*dt*math.sin(theta),F*dt*math.cos(theta),F*math.sin(theta),direction*F*math.cos(theta)]#des,y_vel,z_vel,y_acc,z_acc                  
                dt=dt+0.00001
                [u1,phi0]= pos_control(a,feedback_position)                

                for i in range(speed):
                        u2 = att_control2D(phi0,feedback_attitude)
                        dt=dt+0.001
                        #***feedback****#
                        x1=state[0]#y-position
                        x2=state[1]#y-vel.
                        x3=state[2]#z-position
                        x4=state[3]#z-vel
                        x5=phi0
                        x6=0#phi0*dt        
                                                                       
                        state= odeint(model_y_phi2,state,[0,dt],args=(x1,x2,x3,x4,x5,x6,u1,u2))[1]
            
                        feedback_attitude=[state[4],state[5]]   
        
                        actual_z= state[2]
                        actual_y=state[0]                                                   

                        dist = math.hypot(q1[0] - actual_y, q1[1] - actual_z) #using the norm 2
                       
                        if ((hover == True) and (elapsed < T)):
                                elapsed = time.time() - start #check time to see if meets desired hovering time                                
                                print("TIME:")
                                print(elapsed)                               
                                if (elapsed >=T):        
                                        return [map_position_y,map_position_z,epoch_list]                                                
                                epoch_list.append(elapsed)    #house keeping to plot in main file
                                time.sleep(0.2)               #required to sleep to prevent overflow
                        elif (dist < tolerance):            
                             inrange= True  
                             if(old_dist < dist): #validate if target can become more accruate 
                                return [map_position_y,map_position_z,epoch_list]
                        map_position_y.append(actual_y)
                        map_position_z.append(actual_z)
                        old_dist= dist                                                          
                        
                feedback_position=[state[0], state[2],state[1],state[3]]  
              
        return [map_position_y,map_position_z,epoch_list]
