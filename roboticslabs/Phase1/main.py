from __future__ import division
from partA_2D import path2D
from partB_3D import path3D
from pylab import *

def main():
        
        (dimensionality) = int(input("Press 2 for 2D or 3 for 3D? "))
               
        if (dimensionality==2):
                        initial_position_y = input("Choose y initial position(q0) for 2D flight:")
                        initial_position_z = input("Choose z initial position(q0) for 2D flight:")
                        qh_y= input("Choose y component for desired hovering pose(qh) for 2D flight:")
                        qh_z= input("Choose z component for desired hovering pose(qh) for 2D flight:")
                        z_t_y= input("Choose a vertial height (zt) for 2D flight:")     
                        q0=[initial_position_y,initial_position_z] #
                        q1=[initial_position_y,z_t_y]# 
                        q2=[qh_y,qh_z]# desired hovering pose
                        q3=[qh_y,0] #destination
                                                     
        elif(dimensionality==3):
                        (initial_position_x)= int(input("Choose x initial position(q0) for 3D flight:"))
                        (initial_position_y)= int(input("Choose y initial position(q0) for 3D flight:"))
                        (initial_position_z)= int(input("Choose z initial position(q0) for 3D flight:"))
                        (qh_x)= int(input("Choose x component for desired hovering pose(qh) for 3D flight:"))
                        (qh_y)= int(input("Choose y component for desired hovering pose(qh) for 3D flight:"))
                        (qh_z)= int(input("Choose z component for desired hovering pose(qh) for 3D flight:"))
                        z_t_y= int(input("Choose a vertial height (zt) for 3D flight:"))
                        q0=[initial_position_x,initial_position_y,initial_position_z] #
                        q1=[initial_position_x,initial_position_y,z_t_y]# 
                        q2=[qh_x,qh_y,qh_z]# desired hovering pose
                        q3=[qh_x,qh_y,initial_position_z] #destination
        T= int(input("Choose T seconds to hover for:"))
  

        if (dimensionality==2):
                                
                [map_p_y,map_p_z,_] = path2D(q0,q1,T)
                q1_actual=[map_p_y[len(map_p_y)-1],map_p_z[len(map_p_z)-1]] 
                [map_p_y1,map_p_z1,_]= path2D(q1_actual,q2,T)         
                q2_actual=[map_p_y1[len(map_p_y1)-1],map_p_z1[len(map_p_z1)-1]]
                [map_p_yh, map_p_zh,time_list] = path2D(q2_actual,q2_actual,T)
                [map_p_y2,map_p_z2,_]= path2D(q2_actual,q3,T) 

                ion()
                fig=plt.figure()
                ax = fig.add_subplot(111)
                plt.xlabel("Y")
                plt.ylabel("Z")
                ax.set_xlim((0, q2[0]+10)) #padding 
                ax.set_ylim((0, q2[1]+10))

                lin=None
                for i in range(len(map_p_y)):
                    time = "time:" + str(i)
                    lin = ax.scatter(map_p_y[i],map_p_z[i])
                    draw()

                pause(0.0000001)
                for i in range(len(map_p_y1)):
                    time = "time:" + str(i)
                    lin = ax.scatter(map_p_y1[i],map_p_z1[i])
                    draw()

                for i in range(len(map_p_yh)):
                    time = " Hovering Time:" + str(time_list[i])+"secs"
                    ax.set_title(time,loc='left')                      
                    lin = ax.scatter(map_p_yh[i],map_p_zh[i])
                    draw()
                    pause(0.0000001)
                time = " Hovering Time:" + str(T)+"secs"
                ax.set_title(time,loc='left')                      
                    
                draw()
                for i in range(len(map_p_y2)):
                    time = "time:" + str(i)
                    lin = ax.scatter(map_p_y2[i],map_p_z2[i])
                    draw()
                
                ioff()
                show()

        elif(dimensionality==(3)):
                
                [map_p_x,map_p_y,map_p_z,_] = path3D(q0,q1,T)
                q1_actual=[map_p_x[len(map_p_x)-1],map_p_y[len(map_p_y)-1],map_p_z[len(map_p_z)-1]] 
                [map_p_x1,map_p_y1,map_p_z1,_]= path3D(q1_actual,q2,T)         
                q2_actual=[map_p_x1[len(map_p_x1)-1],map_p_y1[len(map_p_y1)-1],map_p_z1[len(map_p_z1)-1]]
                [map_p_xh, map_p_yh, map_p_zh,time_list] = path3D(q2_actual,q2_actual,T)
                [map_p_x2,map_p_y2,map_p_z2,_]= path3D(q2_actual,q3,T) 

                ion()
                fig=plt.figure()
        
                ax = fig.add_subplot(111, projection='3d')

                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_xlim((0, q2[0]+10)) #padding 
                ax.set_ylim((0, q2[1]+10))
                ax.set_zlim((0, q2[2]+10))
                
                lin=None
                for i in range(len(map_p_x)):
                    #time = "time:" + str(i)
                    lin = ax.scatter(map_p_x[i],map_p_y[i],map_p_z[i])
                    draw()
                pause(0.0000001)

                for i in range(len(map_p_x1)):
                    #time = "time:" + str(i)
                    lin = ax.scatter(map_p_x1[i],map_p_y1[i],map_p_z1[i])
                    draw()

                for i in range(len(map_p_xh)):
                    time = " Hovering Time:" + str(time_list[i])+"secs"
                    ax.set_title(time,loc='left')                      
                    lin = ax.scatter(map_p_xh[i],map_p_yh[i],map_p_zh[i])
                    draw()
                    pause(0.0000001)
                time = " Hovering Time:" + str(T)+"secs"
                ax.set_title(time,loc='left')                      
                    
                draw()
                for i in range(len(map_p_x2)):
                    #time = "time:" + str(i)
                    lin = ax.scatter(map_p_x2[i],map_p_y2[i],map_p_z2[i])
                    draw()

                ioff()
                show()

        else:
                print ("dimensionality is invalid")
                                      
if __name__== '__main__':
        main()
