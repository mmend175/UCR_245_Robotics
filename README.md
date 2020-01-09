# UCR_245_Robotics

# Phase_One: Obstacle Avoidance Simulator Of Quadrotor's Path 
******************************************************

Run Instructions
******************************************************
Step 1) Run the main.py file.

Step 2) Choose '2' for 2D or '3' for 3D

Step 3) Enter the desired locations for the flight path.
    --> 2D Example (q0_y=0,  q0_z=0,  qh_y=30,  qh_z=120,  zt=100,  T=5)
    --> 3D Example (q0_x=0,  q0_y=0,  q0_z=0,  qh_x=200,  qh_y=200,  qh_z=200,  zt=100,  T=5)

Step 4) Observe plot of the Quadrotor's path.
    --> The paths are plotted as a function of time
    --> As time progresses,  scatter plot points are added,  representing each flight path segment
    --> The final path will not plot until hover time T has completed.
******************************************************

Assumptions / Simplifications
******************************************************
- Our tolerance level is 5% of the euclidean distance between the two points of each path,
therefore we assume that our controller can achieve this level of accuracy. 

- We also assume that acceleration is constant throughout the flight with respect to each control loop iteration.

- We only keep track of time during the hover portion of the flight (ensuring the hover T time is met). 
This meaning that the remainder of the flight consists of paths and not trajectories,  since we did not
constrain the flight paths to reach their end by a certain point in time. 

- In order to keep track of time during the hover portion,  there is a sleep function call allowing us
to sample at a lower rate than the loop. This,  however,  causes the control loop to be slightly slower
than regular flight control. This will be addressed in following project phases.

- Our thrust is assumed to be constant.
******************************************************

# PHASE TWO Motion Planning Using Varies Algorithms 
***************************************************************************************************

We implemented two motion planners. One is A* and the other is Rapidly Exploring Random Trees *(RTT-Star).
We felt two algorithms is better than one. Thus, we will let the user decide on which algorithm to choose.
RTT-Star will perform significantly faster compared to A*. However, A* grabs the optimal path.

Dependencies
**************************************************************************************************
Rtree - (for RRT*) LINUX RECOMMENDED for use(i.e. Vitual Machine, Dual boot)
    > How to install on linux
        sudo apt-get update
        sudo apt-get install curl

        curl -L http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz | tar xz
        cd spatialindex-src-1.8.5
        ./configure
        make
        sudo make install
        sudo ldconfig
    > Windows: Please refer to Website: http://toblerity.org/rtree/install.html#windows

numpy
scipy
matplotlib
time
os, sys
math
**************************************************************************************************


Run Instructions
**************************************************************************************************
Step 0) Insure Dependencies are installed

Step 1) Run the main.py file.

Step 2) When prompted, choose if you are inputting information by a file or manually.
    -- If by file, please follow the format of the 'sample#.txt' files provided with the code
        -- Place file in the same directory as the code files, then when prompted enter the
           file name (ie. 'sample1.txt')

    -- If manually, enter the following Map information:
        -- Map's origin (x, y, z)
        -- Size of the map (width,  length,  height)
        -- Number of obstacles and their information as follows:
            -- Obstacle's origin (x, y, z)
            -- Size of the obstacle (width,  length,  height)
        -- Desired initial quadrotor position (x, y, z)
        -- Desired final quadrotor position (x, y, z)

Step 3) Choose your desired algorithm A* or Rapidly Exploring Random Tree* (RRT-star)

Step 4) The algorithm will run and find an optimal path to the final position

Step 5) Finally a 3D plot will be displayed of the resulting quadrotor path through the obstacles.
    -- The map limits are shown in blue
    -- The obstacles are shown in red
    -- The algorithm's waypoint path is shown in black
    -- The actual flight trajectory is shown in green
**************************************************************************************************


Assumptions / Simplifications
**************************************************************************************************
1) When using the file input:
    -- start position is assumed to be (0, 0, 0)
    -- final position is assumed to be (lim_x, lim_y, lim_z)

2) The map size and obstacle sizes have been normalized to allow the size of the quadrotor to be
   accounted for. (0.092m x 0.092m x 0.029m) --> (3 x 3 x 1)units

3) Assumed that there are no objects that would obstruct takeoff or landing.
**************************************************************************************************


Unresolved Bugs
**************************************************************************************************
1) If quadrotor's start or final position are extremely blocked in, the A* implementation can
    appear to become stuck. While it may eventually find the optimal path, due to our normalization,
    the binary C_space grid has a extremely large number of nodes. (The algorithm works extremely
    efficiently in most other situations)
        -- Examples of this: sample1.txt, sample3.txt, sample4.txt

**************************************************************************************************

Run-times
**************************************************************************************************
Using A* (Limiting factor is building the C_space, specifically scaled by size and number of obstacles)
> sample.txt:  Total Program: ~16.46 seconds    C_Space: ~13.62 seconds    Algorithm Time: ~2.84 seconds
> sample2.txt: Total Program: ~19.39 seconds    C_Space: ~16.80 seconds    Algorithm Time: ~2.13 seconds

Using RRT*
> sample.txt:  Total Program: ~2.25 seconds    C_Space: ~0.80157 seconds    Algorithm Time: ~0.00175 seconds
> sample2.txt: Total Program: ~2.23seconds    C_Space: ~0.69970 seconds    Algorithm Time: ~0.002023 seconds

**************************************************************************************************

Relevant Material
**************************************************************************************************
> EE245 Course Slides
> Our 3D A* was inspired by the 2D A* implementation found here:
        https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
**************************************************************************************************
Notes
**************************************************************************************************
The take-off will take the map size into a count take off vertical 1/5 the size of the map size z-axis.
We also include 4 png images to the zip file label sampleX.png.(i.e sample1.png) for the sample.txt output.




