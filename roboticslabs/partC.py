import sys
import partA_2D
def main():
         
        dimensionality = input("Press 1 for 2D or 2 for 3D? ")
        if (dimensionality==1):                        
                        initial_position_x = input("Choose x initial(q0) position for 2D:")
                        initial_position_y = input("Choose y initial(q0) position for 2D:")
                        qh_x= input("Choose x (qh) position for 2D:")
                        qh_y= input("Choose y (qh) position for 2D:")
                        z_t_x= input("Choose x component(zt) position for 2D:")
                        z_t_x= input("Choose y component(zt) position for 2D:")                        
                        q0=[initial_position_x,initial_position_y] #
                        q1=[qh_x,qh_y]#
                        q2=[zt_x,zt_y]#
                        q3=[zt_x,0]

                        map_p_y,map_p_z= path(q0,q1,T,True)                                 
        elif(dimensionality==2):                        
                        initial_position_x = input("Choose x initial(q0) position for 3D:")
                        initial_position_y = input("Choose y initial(q0) position for 3D:")
                        initial_position_z = input("Choose z initial(q0) position for 3D:")
                        qh_x= input("Choose x initial(qh) position for 3D")
                        qh_y= input("Choose y initial(qh) position for 3D")
                        qh_z= input("Choose y initial(qh) position for 3D")
                        z_t_x= input("Choose x component(zt) position for 3D")
                        z_t_y= input("Choose y component(zt) position for 3D")
                        z_t_z= input("Choose z component(zt) position for 3D")
                        q0=[initial_position_x,initial_position_y,initial_position_z] #
                        q1=[qh_x,qh_y,qh_z]#
                        q2=[zt_x,zt_y,zt_z]#
                        q3=[zt_x,zt_y,0]
                      
        T= input("Choose  T seconds to hover for:")
                               
        map_p_y,map_p_z= path(q0,q1) 

        map_p_y1,map_p_z1= path(q1,q2) 
        
        map_p_y2,map_p_z2= path(q2,q3) # choose acceleration  to be constant
        plt.plot(map_p_y,map_p_z,'-r',map_p_y1,map_p_z1,'-g',map_p_y2,map_p_z2,'-k') # choose acceleration  to be constant)
        #plt.plot(map_p_y2,map_p_z2)
        plt.xlabel("Y")
        plt.ylabel("Z")
        ax = plt.gca()
        ax.set_autoscale_on(False)
        plt.show()
        plt.xlim((0,200))                
                        
if __name__== '__main__':
        main()
