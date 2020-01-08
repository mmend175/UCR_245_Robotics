from __future__ import division
from time import sleep
import math 
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
G=-9.8 #m/s^2
M = 0.030 #kg
KF=6.11*1e-8 #N/rpm^2
KM=1.5*1e-9 #N*m/rpm^2

#attitude controls gains
kdroll=0
kproll=5
kdpitch=0
kppitch=5
kdyaw=0
kpyaw=2

#altitude  control gains
kdx=0.1
kpx=2
kdz=0#0.3
kpz=2 # 0.8 
kdy=0 
kpy=2
speed=10
I_moment= [[1.43*1e-5,0,0],[0,1.43*1e-5,0,0],[0,0,2.89*1e-5]]
   


'''
        @params: input_vector:(desired position values) 
                [x,y,z,x_vel,y_vel,z_vel,x_acc,y_acc,z_acc]   
                feedback:(actual position values)                   
                [x,y,z,x_vel,y_vel,z_vel]

'''

def pos_control3D(input_vector,feedback):                
              #TODO: Check equation

        u1=M*(G+(kdz*(input_vector[5]-feedback[5])+kpz*(input_vector[2]-feedback[2]))) #altitude(thrust)
        roll=(-1/G)*((input_vector[7]+kdy*(input_vector[4]-feedback[4]))+kpy*(input_vector[1]-feedback[1])) #y
        pitch=(-1/G)*((input_vector[6]+kdx*(input_vector[3]-feedback[3]))+kpx*(input_vector[0]-feedback[0]))#x
        

        return u1,roll,pitch


'''
        @param: feedback: feedback_attitude=[phi,phi',theta,theta',yaw,yaw']
'''
def att_control3D(roll,roll_vel,pitch,pitch_vel,yaw,yaw_vel,feedback):
        u2_=np.zeros(3)               
        u2_[0]= I_moment[0][0]*(kdroll*(roll_vel-feedback[1])+kproll*(roll-feedback[0]))
        u2_[1]= I_moment[1][1]*(kdpitch*(pitch_vel-feedback[3])+kppitch*(pitch-feedback[2]))
        u2_[2]= I_moment[2][2]*(kdyaw*(yaw_vel-feedback[5])+kpyaw*(yaw-feedback[4]))
                         
        return u2_



'''
        @param: X: 
                        x1=#x 
                        x2=#x'
                        x3=#y      
                        x4=#y'
                        x5=#z
                        x6=#z'
                        x7=#roll
                        x8=#roll'
                        x9=#pitch
                        x10#pitch'
                        x11=#yaw
                        x12=yaw'

'''
                
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
                
def path3D(q0,q1,dimensionality):
        dt=0    
        u1=0 
        old_dist=0
        #thrush 
        print("----------------------------------------") 
        print("qo") 
        print(q0) 
        print("q1") 
        print(q1)         
        map_position_z =[]
        map_position_y =[]
        map_position_x =[]
        state=[q0[0],0,q0[1],0,q0[2],0,0,0,0,0,0,0]
        #state_var=[0,0,0,0,0,0]        
        z=[q0[2],0]
        u2=[0,0,0]
        feedback_attitude=[0,0,0,0,0,0]       
        feedback_position=[q0[0],q0[1],q0[2],0,0,0]
        actual_z=0
        actual_y=0
        actual_x=0

        '''
        x = q1[0]-q0[0]
        y=  q1[1]-q0[1]
        z=  q1[2]-q0[2]
        roll_desired=-math.atan2(y,z)
        pitch_desired=math.atan2((x),math.sqrt((y*y+z*z)))
        yaw_desired=0
        
        '''

        diff_roll=[q1[2]-q0[2],q1[1]-q0[1]] 
        if(diff_roll[0]==0):
                roll_desired = 0
                print("NO!!!!-phi")
                print(diff_roll)



        else:

                d=diff_roll[1]/diff_roll[0]
                roll_desired=math.atan(d)



        diff_pitch=[q1[2]-q0[2],q1[0]-q0[0]]

        if(diff_pitch[0]==0):

                pitch_desired = 0

                print("NO!!!!-pitch")
                print(diff_pitch)

        else:

                d=diff_pitch[1]/diff_pitch[0]
                pitch_desired=math.atan(d)

        diff_yaw=[q1[1]-q0[1],q1[0]-q0[0]]
        if(diff_yaw[0]==0):
                yaw_desired = 0
                print("NO!!!!-yaw")
                print(diff_yaw)



        else:

                d=diff_yaw[1]/diff_yaw[0]

                yaw_desired=math.atan(d)

        print("yaw_desired")
        print(yaw_desired)
        print("pitch_desired")
        print(pitch_desired)
        print("roll_desired")
        print(roll_desired)
#        F = (10)/(math.sin(phi_desired)+math.sin(pitch_desired)) #choose value greater than >0 to a pose gravity         
        F=0.2
        inrange=False
        print(F*dt*(math.sin(yaw_desired)+math.sin(yaw_desired)))
        dist = math.sqrt((q1[0] - q0[0])**2+(q1[1] - q0[1])**2+(q1[2] - q0[2])**2) 
        tolerance = 9
        print(dist) #using the norm 2 distance to see if quadrotor is close to destination
        while(dist > tolerance or inrange):
               
               print("input vector-outer")    
               #---------desired input vector-----------------------#

               if (diff_roll[0] < 0):
                        direction = -1
               else:
                        direction = 1


                
               a=[q1[0],q1[1],q1[2], #desired position x,y,z

    F*dt*(math.sin(pitch_desired)+math.sin(yaw_desired)),  #x-vel
               F*dt*(math.sin(roll_desired)+math.sin(yaw_desired)), #y-vel
               F*dt*(math.cos(pitch_desired)+math.cos(roll_desired)),#z-vel

               F*(math.sin(pitch_desired)+math.sin(yaw_desired)),#acc-x
               F*(math.sin(roll_desired)+math.sin(yaw_desired)),#acc-y
               F*direction*(math.cos(pitch_desired)+math.cos(roll_desired))]#acc-z
               dt=dt+0.001 
               
               print(a)
               [u1,roll,pitch,]= pos_control3D(a,feedback_position)  

               print("time")
               print(dt)   
               print("u1")                                   
               print(u1)                                   
               print("roll")                                   
               print(roll)                                   
               print("pitch")                                   
               print(pitch)
                                               
               for i in range(speed):
                 #TODO: check if z should be inside the loop or not???

                        roll_vel = math.cos(pitch)*state[7]-math.cos(roll)*math.sin(pitch)*state[11]                        
                        pitch_vel = state[9]+math.sin(roll)*state[11]                        
                        yaw_vel = math.sin(pitch)*state[9]+math.cos(roll)*math.cos(pitch)*state[11] 

                        print("roll_vel")
                        print(roll_vel)
                        print("pitch_vel")
                        print(pitch_vel)
                        print("yaw_vel")
                        print(yaw_vel)
                       
                        print("time inner")
                        print(dt)  
                        u2[0],u2[1],u2[2] = att_control3D(roll,roll_vel,pitch,pitch_vel,yaw_desired,yaw_vel,feedback_attitude)

                        print("U2")
                        print(u2)                           
                        #***feedback****#

                        x1=state[0] #x 
                        x2=state[1] #x'
                        x3=state[2]#y      
                        x4=state[3]#y'
                        x5=state[4]#z
                        x6=state[5]#z'
                        x7=roll#roll
                        x8=0 #roll'
                        x9=pitch#pitch
                        x10=0#pitch'
                        x11=yaw_desired#yaw
                        x12=0#yaw'
                        state= odeint(model_state,state,[0,dt],args=(x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,u1,u2))[1]
                        print("state")                      
                        print(state)



                        feedback_attitude=[state[6],state[7],state[8],state[9],state[10],state[11]]#angle and angle velocity 


                        actual_x=state[0]                       
                        actual_xvel=state[1]
                        actual_y=state[2]
                        actual_yvel=state[3]                                         
                        actual_z=state[4]                       
                        actual_zvel=state[5]

                        print("feedback_attitude")
                        print(feedback_attitude)

                        dist = math.sqrt((q1[0] - actual_x)**2+(q1[1] - actual_y)**2+(q1[2] -actual_z)**2) #using the norm 2
                        print("dist")
                        print(dist)
                        if (dist < tolerance or inrange==True):            
                             inrange= True  
                             if(old_dist < dist): #validate if target can become more accurate 
                                return [map_position_x,map_position_y,map_position_z]
                        map_position_y.append(actual_y)
                        map_position_z.append(actual_z)
                        map_position_x.append(actual_x)
                        old_dist= dist                                                          
                        feedback_position=[actual_x,actual_y,actual_z,actual_xvel,actual_yvel,actual_zvel]  #feed_back x,y, w/ their vels                        
                        print("position")          
                        print(feedback_position) 
                        dt=dt+0.001                
                        
               print("position")          
               print(feedback_position)
               
              
               '''
               x1=state[0] #x 
               x2=state[1]# x'
               x3=state[2]#y      
               x4=state[3]#y'
               x5=state[4]#z                                                
               x6=state[5]#z'
               x7=state_var[0]#roll
               x8=state_var[2]#pitch

               state= odeint(model_state_position,state,[0,dt],args=(x1,x2,x3,x4,x5,x6,x7,x8,u1))[1]
               '''
        return [map_position_x,map_position_y,map_position_z]
                        
