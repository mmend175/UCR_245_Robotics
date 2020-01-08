Skip
to
content

Why
GitHub?




Enterprise
Explore

Marketplace
Pricing

Sign in
Sign
up

2
0

2

dani199221 / ENGR - E599
Code
Issues
0
Pull
requests
0
Projects
0
Security
Insights
Join
GitHub
today

GitHub is home
to
over
36
million
developers
working
together
to
host and review
code, manage
projects, and build
software
together.
ENGR - E599 / python / controller.py


@dani199221


dani199221
Final
Commit
e31a28a
on
Dec
6, 2017
139
lines(108
sloc) 4.79
KB
import numpy as np
import scipy.integrate as integrate
import math
from math import sin, cos, tan, exp

e1 = np.array([1, 0, 0])
e2 = np.array([0, 1, 0])
e3 = np.array([0, 0, 1])
mass = 4.34  # kg
g = 9.81  # m/s2

# The inertia matrix
J = I = np.array([[0.0820, 0, 0], \
                  [0, 0.0845, 0], \
                  [0, 0, 0.1377] \
                  ])
ctf = 8.004e-4  # km/kf
d = 0.315


class Controller:
    def __init__(self):
        self.k_x = 16
        self.k_v = 5.6
        self.k_r = 8.81
        self.k_w = 2.54
        self.prev_Rd = np.zeros((3, 3))
        self.prev_wd = np.zeros(3)
        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.ad = np.zeros(3)
        self.e_x = 0
        self.e_v = 0
        self.e_R = 0
        self.e_w = 0

    def get_xd(self):
        return self.xd

    def get_vd(self):
        return self.vd

    def get_errors(self):
        return self.e_x, self.e_v, self.e_R, self.e_w

    # returns the desired position, velocity and acceleration
    def get_x_desired(self, t):
        self.xd = np.array([0.4 * t, 0.4 * sin(np.pi * t), 0.6 * cos(np.pi * t)])
        self.vd = np.array([0.4, 0.4 * np.pi * cos(np.pi * t), -0.6 * np.pi * sin(np.pi * t)])
        self.ad = np.array([0, -0.4 * np.pi * np.pi * sin(np.pi * t), -0.6 * np.pi * np.pi * cos(np.pi * t)])
        return self.xd, self.vd, self.ad

    # returns the desired b1d, b2d, b3d
    def get_Rd(self, t, e_v, e_x, ad):
        b1d = np.array([cos(np.pi * t), sin(np.pi * t), 0])

        # b3d calculation
        res = -self.k_x * e_x - self.k_v * e_v - mass * np.array([0, 0, g]) + mass * ad
        b3d = -1 * res / np.linalg.norm(res)  # get b3d from eq 12

        # b2d calculation
        b2d = np.cross(b3d, b1d) / np.linalg.norm(np.cross(b3d, b1d))  # get b2d from b3d and b1d

        # b1d calculation for R matrix
        new_b1d = np.cross(b2d, b3d)  # get new b1d from corss product of b2d and b3d

        Rd = np.array([[b1d[0], b2d[0], b3d[0]], [b1d[1], b2d[1], b3d[1]], [b1d[2], b2d[2], b3d[2]]])
        print
        Rd
        return Rd

    # returns the desired Force required
    def getF(self, e_x, e_v, ad, R):
        a = -1 * (-self.k_x * e_x - self.k_v * e_v - mass * np.array([0, 0, g]) + mass * ad)
        return np.dot(a, np.dot(R, e3))

    def getM(self, wd, e_R, e_w, w, Rd, dt, R):
        wd_dot = (wd - self.prev_wd) / dt
        self.prev_wd = wd
        first = -1 * self.k_r * e_R - self.k_w * e_w
        second = np.cross(w, np.dot(I, w))
        a = np.dot(self.hat_map(w), R.transpose())
        b = np.dot(Rd, wd)
        c = np.dot(np.dot(R.transpose(), Rd), wd_dot)
        third = -1 * np.dot(J, np.dot(a, b) - c)

        M = first + second + third
        return M

    # returns the desired F and M to the main function
    def update(self, curr_state, t, dt):
        # get desired position, velocity and acceleration
        xd, vd, ad = self.get_x_desired(t)
        # get e_x e_v
        self.e_x = curr_state[0:3] - xd
        self.e_v = curr_state[3:6] - vd

        # get the desired heading directions for the the body frame
        Rd = self.get_Rd(t, self.e_v, self.e_x, ad)

        R = self.rotation_matrix(curr_state[6:9])  # get rotation matrix from euler angles

        # get e_R equation (10)
        self.e_R = 0.5 * self.vee_map(np.dot(Rd.T, R) - np.dot(R.T, Rd))

        # get e_w and pre requisites
        w = curr_state[9:12]
        Rd_dot = (Rd - self.prev_Rd) / dt
        self.prev_Rd = Rd
        wd = self.vee_map(np.dot(np.linalg.inv(Rd), Rd_dot))

        # get e_w equation (11)
        self.e_w = w - np.dot(np.dot(R.transpose(), Rd), wd)

        # calculate f
        F = self.getF(self.e_x, self.e_v, ad, R)
        # calculate M

        M = self.getM(wd, self.e_R, self.e_w, w, Rd, dt, R)

        return F, M

    def vee_map(self, mat):  # converts a 3x3 matrix to a 3x1 matrix
        return np.array([mat[2][1], mat[0][2], mat[1][0]])

    # https://www.wikiwand.com/en/Skew-symmetric_matrix
    def hat_map(self, mat):  # returns the 3x3 skew symmetric matrix of the rotation matrix which is 3x1
        return np.array([[0, -1 * mat[2], mat[1]],
                         [mat[2], 0, -1 * mat[0]],
                         [-1 * mat[1], mat[0], 0]])

    def rotation_matrix(self, state):  # inverse of matrix is its transpose
        phi, theta, sy = state
        return np.array([[cos(sy) * cos(theta), cos(sy) * sin(theta) * sin(phi) - sin(sy) * cos(phi),
                          cos(sy) * sin(theta) * cos(phi) + sin(sy) * sin(phi)], \
                         [sin(sy) * cos(theta), sin(sy) * sin(theta) * sin(phi) + cos(sy) * cos(phi),
                          sin(sy) * sin(theta) * cos(phi) - cos(sy) * sin(phi)], \
                         [-1 * sin(theta), cos(theta) * sin(phi), cos(theta) * cos(phi)] \
                         ])



