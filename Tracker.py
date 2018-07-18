import sys
sys.path.append("../../PathPlanning/CubicSpline/")

import math
import matplotlib.pyplot as plt
# import cubic_spline_planner
from math import sqrt
from sys import argv
import argparse
# import myFunc as mf
# import ompl_demo as odm
import matplotlib.pyplot as plt
import matplotlib
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
import numpy as np
from numpy import *
from scipy.optimize import least_squares
from scipy.optimize import minimize
from numpy import linalg as LA
from numpy.linalg import inv
from scipy import interpolate
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
# import cubic_spline_planner
from scipy.interpolate import spalde
from scipy.interpolate import splev
from scipy.interpolate import splrep
from scipy.interpolate import spline
import random
import copy
import sympy

NUM_ROBOTS = 3
gamma = math.pi/6
flag = 0
max_vel = 4
max_vel_actual = 0.3
bot_rad = 0.13
target_speed = 0.008
# Start = [0,0]
# Goal = [-2.5,3]
circularObstacles = [(5, 2.5, 2),(1,6,2.005),(9,4,2)]

# path_feasible = False 
safety_margin = 0.3 # gap between robots to avoid collision
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""


k = 0.2 #1 #    0.5  # control gain
Kp = 0.7  # speed propotional gain
L = 0.4  # [m] Wheel base of vehicle
Kdis = 0.07
max_steer = math.radians(30.0)  # [rad] max steering angle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta,dt,position):

    if delta >= max_steer:
        delta = max_steer
    elif delta <= -max_steer:
        delta = -max_steer

    state.x = position.x
    state.y = position.y
    state.yaw = position.theta
    # state.x = state.x + state.v * math.cos(state.yaw) * dt
    # state.y = state.y + state.v * math.sin(state.yaw) * dt
    # state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    # state.yaw = pi_2_pi(state.yaw)
    state.v = state.v + a * dt
    # state.v = a * dt

    return state


def PIDControl(target, current,index,state,gen_x,gen_y):
    a = Kp * (target - current) 
    dis = sqrt((state.x - gen_x[index])**2 + (state.y - gen_y[index])**2)
    a = a + Kdis * dis

    return a


def stanley_control(state, cx, cy, cyaw, pind):

    ind, efa = calc_target_index(state, cx, cy,pind)

    if pind >= ind:
        ind = pind

    theta_e = pi_2_pi(cyaw[ind] - state.yaw)
    theta_d = math.atan2(k * efa, state.v)
    delta = theta_e + theta_d

    return delta, ind


def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def calc_target_index(state, cx, cy,present_target_ind):

    # calc frant axle position
    fx = state.x + L * math.cos(state.yaw)
    fy = state.y + L * math.sin(state.yaw)

    # search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    # mind = sqrt((fx - cx[present_target_ind])**2 + (fy - cy[present_target_ind])**2)
    ind = d.index(mind)
    # if state.x >= cx[present_target_ind] and state.y >= cy[present_target_ind] :
    # if 
    #     ind = present_target_ind + 1
    # else :
    #     ind = present_target_ind

    tyaw = pi_2_pi(math.atan2(fy - cy[ind], fx - cx[ind]) - state.yaw)
    if tyaw > 0.0:
        mind = - mind

    return ind, mind

