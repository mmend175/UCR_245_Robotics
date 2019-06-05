#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser

if __name__ == '__main__':

    index = 1   # for cf1
    initialPosition = [0,-1.5,0] # x,y,z coordinate for this crazyflie
    #desired = [-2.0,0.0,0] # x,y,z coordinate for this crazyflie
    #desired2= [2.0,5.0,0] # x,y,z coordinate for this crazyflie
    cfs = CrazyflieParser(index, initialPosition)
    cf = cfs.crazyflies[0]
    time = cfs.timeHelper

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    #cf.setParam("ring/effect", 7)

    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    time.sleep(3.0)
    size= 100
   
   
    desired=[]
    '''
    cf.goTo(goal=[0,-1,0.5],yaw = 0, duration=3.0)
    time.sleep(3.0)
    cf.goTo(goal=[1,-1,0.5],yaw = 0, duration=3.0)
    time.sleep(3.0)
    '''
   #2d figure 8 
    '''
    time_dt = np.linspace(0,2*math.pi,size) #2d 8 fig.
    for dt in range(size) :
        x = 0.5* math.sin(time_dt[dt])
        y = 0.5* (math.sin(time_dt[dt])*math.cos(time_dt[dt]))-1.5
        desired.append([x,y,0.5])
    print(time_dt)
    
   '''
   #3d fig 8 
    time_dt = np.linspace(0,2*math.pi,size) #2d 8 fig.
    z_dt = np.linspace(0.5,1.3,size) #2d 8 fig.
    for dt in range(size) :
        x = 0.5* math.sin(time_dt[dt])
        y = 0.5* (math.sin(time_dt[dt])*math.cos(time_dt[dt]))-1.5
        z = z_dt[dt] 
        desired.append([x,y,z])
    print(time_dt)

    '''
    #2d circle
    time_dt = np.linspace(math.pi,3*math.pi,size)  #time for circle
    #x^2+y^2 = r  x  =cos(t), y = sin(t)
    for dt in range(len(time_dt)) :
        y = 0.5* math.sin(time_dt[dt])-1.5
        x = 0.5* math.cos(time_dt[dt])+0.5
        desired.append([x,y,0])
    '''
    
    for dt in range(size) :
        cf.cmdPosition(pos=desired[dt],yaw=0)    
        print(desired[dt])
        time.sleep(0.002)
    
    # FILL IN YOUR CODE HERE
    # Please try both goTo and cmdPosition

    cf.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(5.0)
