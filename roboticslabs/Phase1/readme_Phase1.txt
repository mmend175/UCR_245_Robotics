******************************************************
Class: EE245 Advanced Robotics
Professor: Konstantinos Karydis
Title: Robotics Quadrotor,  Project Phase 1
Authors: Matthew Mendoza and Benjamin Styerwalt
Date: May 13th,  2019
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
> Our tolerance level is 5% of the euclidean distance between the two points of each path,
therefore we assume that our controller can achieve this level of accuracy. 

> We also assume that acceleration is constant throughout the flight with respect to each control loop iteration.

> We only keep track of time during the hover portion of the flight (ensuring the hover T time is met). 
This meaning that the remainder of the flight consists of paths and not trajectories,  since we did not
constrain the flight paths to reach their end by a certain point in time. 

> In order to keep track of time during the hover portion,  there is a sleep function call allowing us
to sample at a lower rate than the loop. This,  however,  causes the control loop to be slightly slower
than regular flight control. This will be addressed in following project phases.

> Our thrust is assumed to be constant.
******************************************************


Unresolved Bugs
******************************************************
> Murphy's Law (You never know for sure)
******************************************************

Relevant Material
******************************************************
> EE245 Course Slides
******************************************************