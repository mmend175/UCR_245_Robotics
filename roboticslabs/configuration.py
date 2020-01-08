
import numpy as np
#robot dimensions
#.092X0.92X0.92 (mm)
robot_width=92
robot_length=92
robot_height=29 


def configure(map_size, obstacles)

        for i in range(obstacles):
                obs_array_aug[i] = [obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3]+robot_width, obstacles[i][4]+robot_length,obstacles[i][5]+robot_height]


        environment= np.ones((map_size[3],map_size[4],map_size[5]))
        
        for i in range(map_size[3]): #width of map
                                
                if( obstacles[i][0 )
                for j in range(map_size[4]): #length of map 
                        for k in range(map_size[5]):#height of map 

        
