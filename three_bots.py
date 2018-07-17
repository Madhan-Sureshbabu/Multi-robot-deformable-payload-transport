#!/usr/bin/env python
# from ompl import util as ou
# from ompl import base as ob
# from ompl import geometric as og
# import cvxopt as cvx
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
import Planner as pln
import Tracker as tr
from geometry_msgs.msg import Pose2D 
import rospy
import time

def get_position_bot0(msg):
    global position_bot0,Start,Goal
    position_bot0 = msg
    Start = [position_bot0.x,position_bot0.y]
    Goal = [-2,3]


def get_position_bot1(msg):
    global position_bot1,Start,Goal
    position_bot1 = msg
    # Start = [position.x,position.y]
    # Goal = [-2,3]


def get_position_bot2(msg):
    global position_bot2,Start,Goal
    position_bot2 = msg
    # Start = [position.x,position.y]
    # Goal = [-2,3]


def ros_init():
    try :
        rospy.init_node('three_bots')
        # turtlename = rospy.get_param('~turtle')
        rospy.Subscriber('/poseRPY0',Pose2D,get_position_bot0,queue_size=1)
        rospy.Subscriber('/poseRPY1',Pose2D,get_position_bot1,queue_size=1)
        rospy.Subscriber('/poseRPY2',Pose2D,get_position_bot2,queue_size=1)
        # pub = rospy.Publisher('2D_pose',Pose2D)
        # pose_msg = Pose2D()
        # rospy.spin()
        time.sleep(0.5)
    except rospy.ROSInterruptException:
        pass
# Start = [0,0]

position_bot0 = Pose2D()
position_bot1 = Pose2D()
position_bot2 = Pose2D()
ros_init()



NUM_ROBOTS = 3
gamma = math.pi/6
flag = 0
max_vel = 4
max_vel_actual = 0.3
bot_rad = 0.13
target_speed = 0.08
Start = [0,0]
Goal = [10,10]


# path_feasible = False 
safety_margin = 0.3 # gap between robots to avoid collision
show_animation = True



vel = {} #in M0 frame
vel_global = {} #in global frame
pos = {}
w = {}
Jzz = {}
mass = {}
bot_paths = {}
bot1x,bot1y,bot2x,bot2y = {},{},{},{}
Bot_path = {}
circle = []
lower_bound = {}
upper_bound = {}
y_intersection = {}

for i in range(NUM_ROBOTS):
    mass[i] = 1
    vel_global[i] = {}
    bot_paths[i] = {}
    Bot_path[i]=np.array([[0,0]])
    lower_bound[i] = {}
    upper_bound[i] = {}
    print Bot_path[i]


def velocity_of_VS_vertices(var):
    energy = 0
    for i in range(NUM_ROBOTS):
        A1 = np.array([[0,-k0[j],1],[k0[j],0,0],[0,0,0]])
        pos[i] = np.array([var[i]*math.cos(var[i+NUM_ROBOTS]),var[i]*math.sin(var[i+NUM_ROBOTS]),1])
        vel[i] = speed[j]*np.matmul(A1,pos[i])
        den = var[i]**2 + (1/((k0[j])**2)) - (2*var[i]*math.sin(var[i+NUM_ROBOTS]))/k0[j]
        # print den<0
        w[i] = (LA.norm(vel[i]))/sqrt(den)
        Jzz[i] = mass[i]*((var[i]*math.cos(var[i+NUM_ROBOTS]))**2+(var[i]*math.sin(var[i+NUM_ROBOTS])-(1/k0[j]))**2)
        energy = energy + (LA.norm(vel[i])**2)*((Jzz[i]*(k0[j]**2))/((k0[j]**2)*(var[i]**2)-2*var[i]*k0[j]*math.sin(var[i+NUM_ROBOTS])+1) + mass[i])
    return energy

def axis_transformation_finder(x):
    A = [[math.cos(x[2]),-1*math.sin(x[2]),x[0]],[math.sin(x[2]),math.cos(x[2]),x[1]],[0,0,1]]
    cost_function = []
    cost_function = np.array(cost_function)
    global_reference_frame = np.array([long_path[j][0],long_path[j][1],1])
    local_reference_frame  = np.array([0,0,1])
    global_reference_frame_estimate = np.matmul(A,local_reference_frame)
    a = global_reference_frame - global_reference_frame_estimate
    cost_function = np.append(cost_function,a,axis=0)
    return cost_function 

def curvefit(pathList,length,order=3) :
    x = pathList[:,0]
    y = pathList[:,1]
    points = np.linspace(0,1,len(x))
    new_points = np.linspace(0,1,length)
    x_coeff = np.polyfit(points,x,order)
    y_coeff = np.polyfit(points,y,order)

    func_x = np.poly1d(x_coeff)
    func_y = np.poly1d(y_coeff)
    func_x_dot = func_x.deriv()
    func_y_dot = func_y.deriv()
    func_x_ddot = func_x_dot.deriv()
    func_y_ddot = func_y_dot.deriv()

    new_x = func_x(new_points)
    new_y = func_y(new_points)
    new_x_dot = func_x_dot(new_points)
    new_y_dot = func_y_dot(new_points)
    new_x_ddot = func_x_ddot(new_points)
    new_y_ddot = func_y_ddot(new_points)

    k = (abs(new_x_dot*new_y_ddot-new_y_dot*new_x_ddot)/(((new_x_dot)**2+(new_y_dot)**2)**1.5))
    # print k

    pathList_new = np.empty([new_x.shape[0],pathList.shape[1]])
    pathList_new[:,0] = new_x
    pathList_new[:,1] = new_y
    pathList_new = np.array(pathList_new)
    return pathList,new_x_dot,new_y_dot,k


def bounds_calculator(hom_matrix,j):
    # global path_feasible
    x, y = sympy.symbols("x y", real=True)
    hom_matrix_inv = inv(hom_matrix)
    for r in range(len(pln.circularObstacles)) :
        # obstacle[2] = obstacle[2] + safety_margin
        obstacle = pln.circularObstacles[r]
        a = np.matmul(hom_matrix_inv,np.array([obstacle[0],obstacle[1],1]))
        obs_centre_x = a[0]
        obs_centre_y = a[1]
        sol1 = [0]
        # print "sol1 cond",sol1==[]
        while sol1!=[]: 
            eq1 = sympy.Eq((x-obs_centre_x)**2+(y-obs_centre_y)**2,obstacle[2]**2)
            eq2 = sympy.Eq((x- upper_bound[0][j]*math.cos(math.pi/6))**2+(y- upper_bound[0][j]*math.sin(math.pi/6))**2,bot_rad**2)
            sol1 = sympy.solve([eq1,eq2])
            if sol1!=[]:
                upper_bound[0][j]=upper_bound[0][j]-0.03
        if obstacle[2]**2 - obs_centre_x**2 >= 0 :
            y_intersection1 = obs_centre_y - abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
            y_intersection2 = obs_centre_y + abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
            if obs_centre_y > 0 :
                if y_intersection1 - bot_rad  < 0.5 :#and y_intersection1 - bot_rad - safety_margin > 0:#2*bot_rad:
                    y_var = y_intersection1 - bot_rad
                    while (y_var > 0):
                        eq1 = sympy.Eq((x-obs_centre_x)**2+(y-obs_centre_y)**2,obstacle[2]**2)
                        eq2 = sympy.Eq((x)**2+(y-y_var)**2,(2*bot_rad)**2)
                        sol = sympy.solve([eq1,eq2])
                        if sol==[] :
                            break
                        y_var = y_var - 0.01
                    upper_bound[2][j] = y_var
                    if upper_bound[2][j] <= lower_bound[2] :
                        # path_feasible = False
                        upper_bound[2][j] = lower_bound[2] + 0.00001 #since upperbound > lower bound
                    # else :
                        # path_feasible = True
            elif obs_centre_y < 0 :
                if y_intersection2 + bot_rad > -0.5 :#and y_intersection2 + bot_rad + safety_margin < 0 :#-2*bot_rad :
                    y_var = y_intersection2 + bot_rad
                    while (y_var < 0) :
                        eq1 = sympy.Eq((x-obs_centre_x)**2+(y-obs_centre_y)**2,obstacle[2]**2)
                        eq2 = sympy.Eq((x)**2+(y-y_var)**2,(2*bot_rad)**2)
                        sol = sympy.solve([eq1,eq2])
                        if sol==[] :
                            break
                        y_var = y_var + 0.01
                    upper_bound[1][j] = y_var
                    if upper_bound[1][j] >= -lower_bound[1] :
                        upper_bound[1][j] = -lower_bound[1]-0.00001
                        # path_feasible = False
                    # else :
                        # path_feasible = True
    # print "upper_bound[0]",upper_bound[0][j]
    # print "upper_bound[1]",upper_bound[1][j]
    # print "upper_bound[2]",upper_bound[2][j]
    # print "---"
    # return path_feasible
    # return upper_bound[1][j]
            
    # print upper_bound[1][j]


pathList = pln.main(Start,Goal)

pathList = np.array(pathList) 
# print "len(pathList)",len(pathList)
long_path,vel_x,vel_y,k0 = curvefit(pathList,pathList.shape[0])

speed = [sqrt(vel_x[i]**2+vel_y[i]**2) for i in range(len(vel_y))]


vel_0 = sqrt((vel_x)**2+(vel_y)**2)
w_0 = vel_0 * k0
for i in range(NUM_ROBOTS):
    for j in range(len(long_path)) :
        upper_bound[i][j] = 0.5
        lower_bound[i] = bot_rad + 0.01

guess = np.array([0.31001,.31001,.31001,0,math.pi/2,math.pi/2])
guess2 = np.array([0.001,0])
axis_guess = np.array([0,0,0])





for j in range(1+len(long_path)) :
    if j != len(long_path) :
        axis_solution = least_squares(axis_transformation_finder,axis_guess)
        axis_theta = math.atan2(vel_y[j],vel_x[j])
        A = [[math.cos(axis_theta),-1*math.sin(axis_theta),axis_solution.x[0]],[math.sin(axis_theta),math.cos(axis_theta),axis_solution.x[1]],[0,0,1]]
        bounds_calculator(A,j)
        # if path_feasible == False
        # print "UP1",upper_bound[0][j]
        # print "UP2",upper_bound[1][j]
        # print "UP3",upper_bound[2][j]
        solution =  minimize(velocity_of_VS_vertices, guess,method='SLSQP', bounds = ((lower_bound[0],upper_bound[0][j]),(lower_bound[0],abs(upper_bound[1][j])),(lower_bound[2],abs(upper_bound[2][j])),(0,math.pi/6),(0,math.pi),(math.pi/6,math.pi)))
        # solution = least_squares(velocity_of_VS_vertices,guess,'3-point',bounds=([0.3,0.3,0,-math.pi], [ upper_bound[0][j],abs(upper_bound[1][j]) ,math.pi,0 ]))
        # solution = least_squares(velocity_of_VS_vertices2,guess2,'3-point',bounds=([2*bot_rad, 0 ], [ upper_bound[1][j],math.pi ]))
        # print "solution.x[0] ",solution.x[0]
        # print "solution.x[1] ",solution.x[1]
        # print "radius of curvature ",1/k0[j]
        # print "position = ",long_path[j][0],long_path[j][1]
        # print (1/k0[j])>solution.x[0]
        # if solution.x[2] < solution.x[0] * math.sin(solution.x[0+NUM_ROBOTS]) :

        solution.x[1+NUM_ROBOTS] = -1*solution.x[1+NUM_ROBOTS] # FOR SECOND ROBOT, ACTUAL BETA = -1 * OPTIMIZED BETA
        # print solution.x
        ICR_centre = np.matmul(A,np.array([0,1/k0[j],1]))

        circle.append(plt.Circle((ICR_centre[0],ICR_centre[1]),1/k0[j],color='b',fill=False))#np.append(circle,plt.Circle((ICR_centre[0],ICR_centre[1]),1/k0[j],color='b',fill=False),axis=0)
        for i in range(NUM_ROBOTS) :
            A_vel = np.array([[math.cos(axis_theta),-1*math.sin(axis_theta),0],[math.sin(axis_theta),math.cos(axis_theta),0],[0,0,1]])
            vel_global[i][j] = np.matmul(A_vel,vel[i])
            # if i==0 :
                # bot_paths[i][j] = np.matmul(A,np.array([0.1*math.cos(0),0.1*math.sin(0),1]))
                # Bot_path[i] = np.append(Bot_path[i],np.array([[bot_paths[i][j][0],bot_paths[i][j][1]]]),axis=0)
            # else :
            bot_paths[i][j] = np.matmul(A,np.array([solution.x[i]*math.cos(solution.x[i+NUM_ROBOTS]),solution.x[i]*math.sin(solution.x[i+NUM_ROBOTS]),1]))
            if j<=1 :
                Bot_path[i] = np.append(Bot_path[i],np.array([[bot_paths[i][j][0],bot_paths[i][j][1]]]),axis=0)
            else :
                m1 = ( bot_paths[i][j][1] - bot_paths[i][j-1][1] ) / (bot_paths[i][j][0] - bot_paths[i][j-1][0] )
                m2 = ( bot_paths[i][j-1][1] - bot_paths[i][j-2][1] ) / (bot_paths[i][j-1][0] - bot_paths[i][j-2][0] )
                Bot_path[i] = np.append(Bot_path[i],np.array([[bot_paths[i][j][0],bot_paths[i][j][1]]]),axis=0)


# fig, ax1= plt.subplots()
# fig1,ax2 = plt.subplots()
# fig3,ax3 = plt.subplots()


            # ax2.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='b'))
Bot_path[0] = np.delete(Bot_path[0],0,0)
Bot_path[1] = np.delete(Bot_path[1],0,0)
Bot_path[2] = np.delete(Bot_path[2],0,0)
 
bot1x,bot1y = Bot_path[0].T
bot2x,bot2y = Bot_path[1].T
bot3x,bot3y = Bot_path[2].T
xc,yc = long_path.T
long_x,long_y = long_path.T

xnew = np.linspace(1,6,len(bot1x))


# print len(xnew)
# print len(xnew2)
bot1x = np.asarray(bot1x).squeeze()
bot1y = np.asarray(bot1y).squeeze()
bot2x = np.asarray(bot2x).squeeze()
bot2y = np.asarray(bot2y).squeeze()
bot3x = np.asarray(bot3x).squeeze()
bot3y = np.asarray(bot3y).squeeze()

print "len(bot1x)",len(bot1x)
print "len(bot2x)",len(bot2x)
print "len(bot3x)",len(bot3x)

tck1x = interpolate.splrep(xnew,bot1x,s=0,k=5)
tck1y = interpolate.splrep(xnew,bot1y,s=0,k=5)
tck2x = interpolate.splrep(xnew,bot2x,s=0,k=5)
tck2y = interpolate.splrep(xnew,bot2y,s=0,k=5)
tck3x = interpolate.splrep(xnew,bot3x,s=0,k=5)
tck3y = interpolate.splrep(xnew,bot3y,s=0,k=5)

xnew1 = np.linspace(1,6,2*len(bot1x))

bot1x = interpolate.splev(xnew1,tck1x,der=0)
bot1y = interpolate.splev(xnew1,tck1y,der=0)
bot2x = interpolate.splev(xnew1,tck2x,der=0)
bot2y = interpolate.splev(xnew1,tck2y,der=0)
bot3x = interpolate.splev(xnew1,tck3x,der=0)
bot3y = interpolate.splev(xnew1,tck3y,der=0)

bot1x_dot = interpolate.splev(xnew1,tck1x,der=1)
bot1y_dot = interpolate.splev(xnew1,tck1y,der=1)
bot2x_dot = interpolate.splev(xnew1,tck2x,der=1)
bot2y_dot = interpolate.splev(xnew1,tck2y,der=1)
bot3x_dot = interpolate.splev(xnew1,tck3x,der=1)
bot3y_dot = interpolate.splev(xnew1,tck3y,der=1)

bot1_dot = sqrt((bot1x_dot)**2+(bot1y_dot)**2)
bot2_dot = sqrt((bot2x_dot)**2+(bot2y_dot)**2)
bot3_dot = sqrt((bot3x_dot)**2+(bot3y_dot)**2)

yaw1 = [math.atan2(bot1y_dot[i],bot1x_dot[i]) for i in range(len(bot1x_dot))]
yaw2 = [math.atan2(bot2y_dot[i],bot2x_dot[i]) for i in range(len(bot2x_dot))]
yaw3 = [math.atan2(bot3y_dot[i],bot3x_dot[i]) for i in range(len(bot3x_dot))]
'''
Path tracking
'''

dt = float(format((xnew1[2]-xnew1[1]),'.2f'))  # [s] time difference
state  = tr.State(x=position_bot0.x, y=position_bot0.y, yaw=position_bot0.theta, v=0.0)
state2 = tr.State(x=position_bot1.x,y=position_bot1.y, yaw=position_bot1.theta, v=0.0)
state3 = tr.State(x=position_bot2.x,y=position_bot2.y, yaw=position_bot2.theta, v=0.0)

lastelement = len(bot1x) - 1
b1x = [0.0]
b1y = [0.0]
b2x = [0.0]
b2y = [0.0]
b3x = [0.0]
b3y = [0.0]
y1=[0.0] #yaw
y2=[0.0] #yaw
y3=[0.0] #yaw

v1=[0.0]
v2=[0.0]
v3=[0.0]
o1 = [0.0]
o2 = [0.0]
o3 = [0.0]
target_ind1, mind = tr.calc_target_index(state, bot1x, bot1y,0)
target_ind2, mind = tr.calc_target_index(state2, bot2x, bot2y,0)
target_ind3, mind = tr.calc_target_index(state3, bot3x, bot3y,0)

rrt = pln.RRT(start=[0, 0], goal=[10, 10],
              randAreax=[-2, 15],randAreay=[-2,15], obstacleList=pln.circularObstacles)
    
error = 0.3
# while (lastelement  > target_ind1  or lastelement > target_ind2 ) and not (target_ind1==lastelement and target_ind2==lastelement) or :
while ((abs(bot1x[lastelement] - b1x[len(b1x)-1])>error) or (abs(bot1y[lastelement]-b1y[len(b1y)-1])>error) or (abs(bot2x[lastelement] - b2x[len(b2x)-1])>error) or (abs(bot2y[lastelement]-b2y[len(b2y)-1])>error) or (abs(bot3x[lastelement] - b3x[len(b3x)-1])>error) or (abs(bot3y[lastelement]-b3y[len(b3y)-1])>error)) :
    di, target_ind1 = tr.stanley_control(state, bot1x, bot1y, yaw1, target_ind1)
    di2, target_ind2 = tr.stanley_control(state2, bot2x, bot2y, yaw2, target_ind2)
    di3, target_ind3 = tr.stanley_control(state3, bot3x, bot3y, yaw3, target_ind3)
    target_ind1 = min(target_ind1,target_ind2,target_ind3)
    target_ind2 = target_ind1
    target_ind3 = target_ind1
    ai = tr.PIDControl(target_speed, state.v,target_ind1,state,bot1x,bot1y)
    state = tr.update(state, ai, di,dt)

    ai2 = tr.PIDControl(target_speed, state2.v,target_ind2,state2,bot2x,bot2y)
    state2 = tr.update(state2, ai2, di2,dt)

    ai3 = tr.PIDControl(target_speed, state3.v,target_ind3,state3,bot3x,bot3y)
    state3 = tr.update(state3, ai3, di3,dt)

    b2x.append(state2.x)
    b2y.append(state2.y)
    y2.append(state2.yaw)
    v2.append(state2.v)
    o2.append(di)

    b1x.append(state.x)
    b1y.append(state.y)
    y1.append(state.yaw)
    v1.append(state.v)
    o1.append(di)

    b3x.append(state3.x)
    b3y.append(state3.y)
    y3.append(state3.yaw)
    v3.append(state3.v)
    o3.append(di3)

    # print "b2",b2x[len(b2x)-1],b2y[len(b2y)-1]
    # print "b1",b1x[len(b1x)-1],b1y[len(b1y)-1]
    # print "b3",b3x[len(b3x)-1],b3y[len(b3y)-1]
    list_length = len(b1x) - 1
    points = [[b1x[list_length],b1y[list_length]],[b2x[list_length],b2y[list_length]],[b3x[list_length],b3y[list_length]]]

    if show_animation:
        plt.cla()
        # plt.plot(bot1x, bot1y, ".r", label="course")
        # plt.plot(bot2x, bot2y, ".r", label="course")
        # plt.plot(bot3x, bot3y, ".r", label="course")
        # plt.plot(b1x, b1y, "-b", label="trajectory")
        # plt.plot(b2x, b2y, "-r", label="trajectory")
        # plt.plot(b3x, b3y, "-g", label="trajectory")

        polygon = plt.Polygon(points,closed=None,fill='g')
        plt.gca().add_patch(polygon)
        plt.axis('scaled')
        plt.plot(b1x[list_length], b1y[list_length], "xb", label="current")
        plt.plot(b2x[list_length], b2y[list_length], "xr", label="current")
        plt.plot(b3x[list_length], b3y[list_length], "xg", label="current")

        plt.plot(Goal[0],Goal[1],"xr",label="goal")

        # plt.plot(bot1x[target_ind1], bot1y[target_ind1], "xb", label="target")
        # plt.plot(bot2x[target_ind2], bot2y[target_ind2], "xr", label="target")
        # plt.plot(bot3x[target_ind3], bot3y[target_ind3], "xg", label="target")
        # plt.plot(xc,yc,".g")
        # plt.show()
        for obstacles in pln.circularObstacles:
                    # ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='b'))
                    rrt.PlotCircle(obstacles[0], obstacles[1], obstacles[2])
        # ax.plot()
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)


# while lastelement >= target_ind :


#     if show_animation:
#         plt.cla()
#         plt.axis("equal")
#         plt.grid(True)
#         plt.pause(0.001)

print "len(v1)",len(v1)
print "len(o1)",len(o1)
# print "v1"
# print v1
max_vel1 = 0
max_vel2 = 0
max_vel3 = 0
for i in range(len(v1)) :
    if v1[i] > max_vel1 : 
        max_vel1 = v1[i]
    if v2[i] > max_vel2 :
        max_vel2 = v2[i]
    if v3[i] > max_vel3 :
        max_vel3 = v3[i]
    if v1[i]>0.3 :
        print v1[i]

print "max_vel1 :",max_vel1
print "max_vel2 :",max_vel2
print "max_vel3 :",max_vel3

# if show_animation:
#     plt.cla()
#     plt.plot(bot1x, bot1y, ".r", label="course")
#     plt.plot(b1x, b1y, "-b", label="trajectory")
#     plt.plot(bot1x[target_ind], bot1y[target_ind], "xg", label="target")
#     plt.axis("equal")
#     plt.grid(True)
#     plt.plot(bot2x, bot2y, ".r", label="course2")
#     plt.plot(b2x, b2y, "-b", label="trajectory2")
    # plt.plot(bot2x[target_ind], bot2y[target_ind], "xg", label="target")


# assert lastelement >= target_ind, "Cannot goal"

    # if show_animation:
    #     plt.plot(cx, cy, ".r", label="course")
    #     plt.plot(x, y, "-b", label="trajectory")
    #     plt.legend()
    #     plt.xlabel("x[m]")
    #     plt.ylabel("y[m]")
    #     plt.axis("equal")
    #     plt.grid(True)

    #     flg, ax = plt.subplots(1)
    #     plt.plot(t, [iv * 3.6 for iv in v], "-r")
    #     plt.xlabel("Time[s]")
    #     plt.ylabel("Speed[km/h]")
    #     plt.grid(True)
    #     plt.show()



'''

phi1 = [math.atan2((bot1y_dot[i]/bot1_dot[i]),(bot1x_dot[i]/bot1_dot[i])) for i in range(len(bot1x_dot))]
phi1 = np.array(phi1)
phi2 = [math.atan2((bot2y_dot[i]/bot2_dot[i]),(bot2x_dot[i]/bot2_dot[i])) for i in range(len(bot2x_dot))]
phi2 = np.array(phi2)

tcko1 = interpolate.splrep(xnew1,phi1,s=0,k=5)
tcko2 = interpolate.splrep(xnew1,phi2,s=0,k=5)

omega1 = interpolate.splev(xnew1,tcko1,der=1)
omega2 = interpolate.splev(xnew1,tcko2,der=1)
# omega1 = np.delete(omega1,len(omega1)-1)
# omega2 = np.delete(omega2,len(omega2)-1)
omega1[0] = 0
omega1[-1] = 0
omega2[0] = 0
omega2[-1] = 0


bot1_dot = (max_vel_actual/max_vel)*bot1_dot
bot2_dot = (max_vel_actual/max_vel)*bot2_dot
'''


# print "bot1_dot"
# print bot1_dot
# print "bot2_dot"
# print bot2_dot
# print "omega1"
# print omega1
# print "omega2"
# print omega2
# print "len(bot1x)",len(bot1x)
# print "len(bot2x)",len(bot2x)
# print "len(omega1)",len(omega1)
# print "len(omega2)",len(omega2)

# xu1,yu1,phi1=np.array([0]),np.array([0]),np.array([0])
# del_t = float(format((xnew1[2]-xnew1[1]),'.2f'))
# for i in range(len(o1)) :
#     phi1 = np.append(phi1,abs(phi1[len(phi1)-1]+(o1[i])*del_t))
#     phi1[len(phi1)-1] = (math.atan2(math.tan(phi1[len(phi1)-1]),1))#+math.pi)
#     xu1=np.append(xu1,xu1[len(xu1)-1]+v1[i]*math.cos(phi1[len(phi1)-1])*del_t)
#     yu1=np.append(yu1,yu1[len(yu1)-1]+v1[i]*math.sin(phi1[len(phi1)-1])*del_t)

# print "xu1"
# print xu1
# print "yu1"
# print yu1
# print "phi1"
# print phi1
# plt.figure()
# plt.plot(xu1,yu1,'o',bot1x,bot1y,'--')


# print "del_t",del_t
# print "phi"
# print phi
# # print "xu"
# # for i in range(len(xu)):
# #     print xu[i]
# # print xu
# print "bot1x"
# print bot1x
# # print "yu"
# # for i in range(len(yu)) :
# #     print yu[i]
# # print yu
# print "bot1y"
# print bot1y

# print "cos(phi)"
# print np.cos(phi)

# print "sin(phi)"
# print np.sin(phi)




'''
ax2.plot(bot1x,bot1y,'-g',bot2x,bot2y,'-b')
for i in range(len(bot1x)):
    # print bot1y[i]
    # ax1.add_patch(plt.Circle((long_x[i],long_y[i]),2*bot_rad,color='b'))
    ax1.add_patch(plt.Circle((bot1x[i],bot1y[i]),bot_rad,color='g'))
    ax1.add_patch(plt.Circle((bot2x[i],bot2y[i]),bot_rad,color='r'))

# plt.plot(bot1x,bot1y,'-g',bot2x,bot2y,'-r')
ax1.plot()
# plt.figure()
# print bot2x,bot2y
'''

plt.show()
exit()
