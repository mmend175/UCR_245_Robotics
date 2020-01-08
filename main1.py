from __future__ import division
import sys
from time import sleep
import math 
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from partA_2D import path2D
from test import path3D
from mpl_toolkits import mplot3d


def main():
        '''
        (dimensionality) = int(input("Press 1 for 2D or 2 for 3D? "))
               
        if (dimensionality==1):
                        initial_position_y = input("Choose y initial position(q0) for 2D flight:")
                        initial_position_z = input("Choose z initial position(q0) for 2D flight:")
                        qh_y= input("Choose y component for desired hovering pose(qh) for 2D flight:")
                        qh_z= input("Choose z component for desired hovering pose(qh) for 2D flight:")
                        z_t_y= input("Choose a vertial height (zt) for 2D flight:")     
                        q0=[initial_position_y,initial_position_z] #
                        q1=[initial_position_y,z_t_y]# 
                        q2=[qh_y,qh_z]# desired hovering pose
                        q3=[qh_y,0] #destination
                                                     
        elif(dimensionality==2):
                        (initial_position_x)= int(input("Choose x initial position(q0) for 3D flight:"))
                        (initial_position_y) = int(input("Choose y initial position(q0) for 3D flight:"))
                        (initial_position_z)= int(input("Choose z initial position(q0) for 3D flight:"))
                        (qh_x)= int(input("Choose x component for desired hovering pose(qh) for 3D flight:"))
                        (qh_y)= int(input("Choose y component for desired hovering pose(qh) for 3D flight:"))
                        (qh_z)= int(input("Choose z component for desired hovering pose(qh) for 3D flight:"))
                        z_t_y= int(input("Choose a vertial height (zt) for 3D flight:"))
                        q0=[initial_position_x,initial_position_y,initial_position_z] #
                        q1=[initial_position_x,initial_position_y,z_t_y]# 
                        q2=[qh_x,qh_y,qh_z]# desired hovering pose
                        q3=[qh_x,qh_z,0] #destination                      
        T= input("Choose  T seconds to hover for:")
        '''                    
        dimensionality=2
        q0=[0,0,0]
        q1=[0,0,10]
        q2=[300,300,400]
        q3=[300,300,0]
        if (dimensionality==1):
                map_p_y,map_p_z= path2D(q0,q1,dimensionality) 
                map_p_y1,map_p_z1= path2D(q1,q2,dimensionality)         
                map_p_y2,map_p_z2= path2D(q2,q3,dimensionality) # choose acceleration  to be constant
                plt.plot(map_p_y,map_p_z,'-r',map_p_y1,map_p_z1,'-g',map_p_y2,map_p_z2,'-k') # choose acceleration  to be constant)     

                plt.xlabel("Y")
                plt.ylabel("Z")
                ax = plt.gca()
                ax.set_autoscale_on(False)
                plt.show()
                plt.xlim((0,200))
        elif(dimensionality==(2)):
                map_p_x,map_p_y,map_p_z= path3D(q0,q1,dimensionality) 
                map_p_x1,map_p_y1,map_p_z1= path3D(q1,q2,dimensionality)         
                map_p_x2,map_p_y2,map_p_z2= path3D(q2,q3,dimensionality) 

      
                fig=plt.figure()
        
                ax = fig.add_subplot(111, projection='3d')
                ax.plot_wireframe(map_p_x,map_p_y,map_p_z) # choose acceleration  to be constant)     
                ax.plot_wireframe(map_p_x1,map_p_y1,map_p_z1) # choose acceleration  to be constant)     
                ax.plot_wireframe(map_p_x2,map_p_y2,map_p_z2) # choose acceleration  to be constant)     
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')

                plt.show()

        else:
                print ("dimensionality is invalid")
        
                                      
if __name__== '__main__':
        main()
