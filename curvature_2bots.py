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
import cubic_spline_planner
from scipy.interpolate import spalde
from scipy.interpolate import splev
from scipy.interpolate import splrep
from scipy.interpolate import spline
import random
import copy
import sympy

"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai(@Atsushi_twi)

"""



show_animation = True

fig, ax= plt.subplots()

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randAreax,randAreay, expandDis=1.0, goalSampleRate=50, maxIter=1000):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrandx = randAreax[0]
        self.maxrandx= randAreax[1]
        self.minrandy = randAreay[0]
        self.maxrandy= randAreay[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrandx, self.maxrandx), random.uniform(
                    self.minrandy, self.maxrandy)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            # print "nind",nind
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList,self.nodeList):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")
        for (x, y, size) in self.obstacleList:
            self.PlotCircle(x, y, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 1))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")
        # ax.add_patch(plt.Circle(x,y,size,'-r'))

    def GetNearestListIndex(self, nodeList, rnd):
    	global flag
    	# print "nodeList"
    	# print [(node.x,node.y) for node in nodeList]
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        dlist_sorted = copy.copy(dlist)
        dlist_sorted.sort()
    	c = np.array([rnd[0],rnd[1]])
    	# print "dlist_sorted",dlist_sorted
    	j=[dlist.index(dlist_sorted[i]) for i in range(len(dlist_sorted))]
        for i in j :
        	# print nodeList[i].x,nodeList[i].y
        	parent_index = nodeList[i].parent
        	# print parent_index
        	# if flag==0 :
        	# 	flag = 1
        	# 	return i
        	if parent_index==None :
				return i
        	else : 
	        	b = np.array([nodeList[i].x,nodeList[i].y])
	        	a = np.array([nodeList[parent_index].x,nodeList[parent_index].y])
	        	p=a-b
	        	q=c-b
	        	# print "p",p
	        	# print "q",q
	        	# print "unit product: ",np.dot(q,p)/(LA.norm(q)*LA.norm(p))
	        	supplementary_angle = math.pi - (math.acos(float(format(np.dot(q,p)/(LA.norm(q)*LA.norm(p)),'.2f'))))
	        	if supplementary_angle <= gamma :
	        		return i
	        	else : 
	        		continue
        # minind = dlist.index(min(dlist))
        # return minind

    def __CollisionCheck(self, node, obstacleList,nodeList):
    	x, y = sympy.symbols("x y", real=True)
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            eq1 = sympy.Eq((x-ox)**2+(y-oy)**2,(size)**2)
            eq2 = sympy.Eq((x-node.x)**2+(y-node.y)**2,(2*bot_rad+0.1)**2)
            sol = sympy.solve([eq1,eq2])
            if d <= size or sol!=[]:#or d1<=0.5+0.1: #0.1 - safety margin
            	# print "collision"
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def GetPathLength(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le


def GetTargetPoint(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen
    #  print(partRatio)
    #  print((ti,len(path),path[ti],path[ti+1]))

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
	# print((x,y))

    return [x, y, ti]


def LineCollisionCheck(first, second, obstacleList):
    # Line Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= (size):
            return False

    #  print("OK")

    return True  # OK


def PathSmoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")

    le = GetPathLength(path)

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        #  print(pickPoints)
        first = GetTargetPoint(path, pickPoints[0])
        # print(first)
        second = GetTargetPoint(path, pickPoints[1])
        # print(second)

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not LineCollisionCheck(first, second, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = GetPathLength(path)

    return path


def main():
    rrt = RRT(start=[0, 0], goal=[10, 10],
              randAreax=[-2, 15],randAreay=[-2,15], obstacleList=circularObstacles)
    path = rrt.Planning(animation=show_animation)
    path.reverse()
    # sine path
    # N = 150
    # n=np.linspace(0,149,50)
    # y_sin = (30 * np.sin(2*math.pi*n/N))
    # path = []
    # for i in range(len(n)):
    # 	path.append([n[i],y_sin[i]])

    # Path smoothing
    maxIter = 1000
    smoothedPath = PathSmoothing(path, maxIter, circularObstacles)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')

        plt.plot([x for (x, y) in smoothedPath], [
            y for (x, y) in smoothedPath], '-b')

        plt.grid(True)
        plt.pause(0.001)  # Need for Mac
        plt.show()
    return path

"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""
import sys
sys.path.append("../../PathPlanning/CubicSpline/")

import math
import matplotlib.pyplot as plt
import cubic_spline_planner


k = 0.5 #1 #    0.5  # control gain
Kp = 1.0  # speed propotional gain
L = 0.6  # [m] Wheel base of vehicle
Kdis = 0.33
max_steer = math.radians(30.0)  # [rad] max steering angle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    elif delta <= -max_steer:
        delta = -max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = pi_2_pi(state.yaw)
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


def velocity_of_VS_vertices(var):
    energy = 0
    for i in range(NUM_ROBOTS):
        A1 = np.array([[0,-k0[j],1],[k0[j],0,0],[0,0,0]])
        pos[i] = np.array([var[i]*math.cos(var[i+NUM_ROBOTS]),var[i]*math.sin(var[i+NUM_ROBOTS]),1])
        vel[i] = speed[j] * np.matmul(A1,pos[i])
        den = var[i]**2 + (1/((k0[j])**2)) - (2*var[i]*math.sin(var[i+NUM_ROBOTS]))/k0[j]
        # print den<0
        w[i] = (LA.norm(vel[i]))/sqrt(den)
        Jzz[i] = mass[i]*((var[i]*math.cos(var[i+NUM_ROBOTS]))**2+(var[i]*math.sin(var[i+NUM_ROBOTS])-(1/k0[j]))**2)
        energy = energy + (LA.norm(vel[i])**2)*((Jzz[i]*(k0[j]**2))/((k0[j]**2)*(var[i]**2)-2*var[i]*k0[j]*math.sin(var[i+NUM_ROBOTS])+1) + mass[i])
    return energy

def velocity_of_VS_vertices2(var):
    energy = 0
    for i in range(NUM_ROBOTS):
        if i==1 :
            A1 = np.array([[0,-k0[j],1],[k0[j],0,0],[0,0,0]])
            pos[i] = np.array([var[i-1]*math.cos(var[i]),var[i-1]*math.sin(var[i]),1])
            vel[i] = np.matmul(A1,pos[i])
            den = var[i-1]**2 + (1/((k0[j])**2)) - (2*var[i-1]*math.sin(var[i]))/k0[j]
            # print den<0
            w[i] = (LA.norm(vel[i]))/sqrt(den)
            Jzz[i] = mass[i]*((var[i-1]*math.cos(var[i]))**2+(var[i-1]*math.sin(var[i])-(1/k0[j]))**2)
            energy = energy + (LA.norm(vel[i])**2)*((Jzz[i]*(k0[j]**2))/((k0[j]**2)*(var[i-1]**2)-2*var[i-1]*k0[j]*math.sin(var[i])+1) + mass[i])

    i=0
    A1 = np.array([[0,-k0[j],1],[k0[j],0,0],[0,0,0]])
    pos[i] = np.array([0.1*math.cos(0),0.1*math.sin(0),1])
    vel[i] = np.matmul(A1,pos[i])
    den = 0.1**2 + (1/((k0[j])**2)) - (2*0.1*math.sin(0))/k0[j]
    # print den<0
    w[i] = (LA.norm(vel[i]))/sqrt(den)
    Jzz[i] = mass[i]*((0.1*math.cos(0))**2+(0.1*math.sin(0)-(1/k0[j]))**2)
    energy = energy + (LA.norm(vel[i])**2)*((Jzz[i]*(k0[j]**2))/((k0[j]**2)*(0.1**2)-2*0.1*k0[j]*math.sin(0)+1) + mass[i])
    return energy

    

def function(x):
    A = [[math.cos(x[2]),-1*math.sin(x[2]),x[0]],[math.sin(x[2]),math.cos(x[2]),x[1]],[0,0,1]]
    cost_function = []
    cost_function = np.array(cost_function)
    global_reference_frame = np.array([long_path[j][0],long_path[j][1],1])
    local_reference_frame  = np.array([0,0,1])
    global_reference_frame_estimate = np.matmul(A,local_reference_frame)
    a = global_reference_frame - global_reference_frame_estimate
    cost_function = np.append(cost_function,a,axis=0)
    return cost_function 

def bounds_calc(hom_matrix,j):
    global path_feasible
    x, y = sympy.symbols("x y", real=True)
    hom_matrix_inv = inv(hom_matrix)
    for r in range(len(circularObstacles)) :
        # obstacle[2] = obstacle[2] + safety_margin
        obstacle = circularObstacles[r]
        a = np.matmul(hom_matrix_inv,np.array([obstacle[0],obstacle[1],1]))
        obs_centre_x = a[0]
        obs_centre_y = a[1] 
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
                    upper_bound[0][j] = y_var
                    if upper_bound[0][j] <= lower_bound[0] :
                        path_feasible = False
                        upper_bound[0][j] = lower_bound[0] + 0.00001 #since upperbound > lower bound
                    else :
                        path_feasible = True
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
                        path_feasible = False
                    else :
                        path_feasible = True
    # print "upper_bound[0]",upper_bound[0][j]
    # print "upper_bound[1]",upper_bound[1][j]



    return path_feasible
    # return upper_bound[1][j]
            
    # print upper_bound[1][j]


NUM_ROBOTS = 2

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
gamma = math.pi/6
flag = 0
max_vel = 4
max_vel_actual = 0.3
bot_rad = 0.13
target_speed = 0.08

for i in range(NUM_ROBOTS):
    mass[i] = 1
    vel_global[i] = {}
    bot_paths[i] = {}
    Bot_path[i]=np.array([[0,0]])
    lower_bound[i] = {}
    upper_bound[i] = {}
    print Bot_path[i]

circularObstacles = [(5, 2.5, 2),(1,6,2.005),(9,4,2)]

path_feasible = False 
safety_margin = 0.3 # gap between robots to avoid collision
pathList = main()

pathList = np.array(pathList) 
# print "len(pathList)",len(pathList)
long_path,vel_x,vel_y,k0 = curvefit(pathList,pathList.shape[0])

speed = [sqrt(vel_x[i]**2 + vel_y[i]**2 ) for i in range(len(vel_y))]


vel_0 = sqrt((vel_x)**2+(vel_y)**2)
w_0 = vel_0 * k0
for i in range(NUM_ROBOTS):
    for j in range(len(long_path)) :
        upper_bound[i][j] = 0.5
        lower_bound[i] = bot_rad + 0.01

guess = np.array([0.31001,.31001,0,math.pi/2])
guess2 = np.array([0.001,0])
axis_guess = np.array([0,0,0])


for j in range(1+len(long_path)) :
    if j != len(long_path) :
        axis_solution = least_squares(function,axis_guess)
        axis_theta = math.atan2(vel_y[j],vel_x[j])
        A = [[math.cos(axis_theta),-1*math.sin(axis_theta),axis_solution.x[0]],[math.sin(axis_theta),math.cos(axis_theta),axis_solution.x[1]],[0,0,1]]
        path_feasible = bounds_calc(A,j)
        solution =  minimize(velocity_of_VS_vertices, guess,method='SLSQP', bounds = ((lower_bound[0],upper_bound[0][j]),(lower_bound[0],abs(upper_bound[1][j])),(0,math.pi),(0,math.pi)))
        # solution = least_squares(velocity_of_VS_vertices,guess,'3-point',bounds=([0.3,0.3,0,-math.pi], [ upper_bound[0][j],abs(upper_bound[1][j]) ,math.pi,0 ]))
        # solution = least_squares(velocity_of_VS_vertices2,guess2,'3-point',bounds=([2*bot_rad, 0 ], [ upper_bound[1][j],math.pi ]))
        # print "solution.x[0] ",solution.x[0]
        # print "solution.x[1] ",solution.x[1]
        # print "radius of curvature ",1/k0[j]
        # print "position = ",long_path[j][0],long_path[j][1]
        # print (1/k0[j])>solution.x[0]
        solution.x[3] = -1*solution.x[3]
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

ax.axis('equal')


            # ax2.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='b'))
Bot_path[0] = np.delete(Bot_path[0],0,0)
Bot_path[1] = np.delete(Bot_path[1],0,0)


bot1x,bot1y = Bot_path[0].T
bot2x,bot2y = Bot_path[1].T
xc,yc = long_path.T
long_x,long_y = long_path.T

xnew = np.linspace(1,6,len(bot1x))


# print len(xnew)
# print len(xnew2)
bot1x = np.asarray(bot1x).squeeze()
bot1y = np.asarray(bot1y).squeeze()
bot2x = np.asarray(bot2x).squeeze()
bot2y = np.asarray(bot2y).squeeze()

print "len(bot1x)",len(bot1x)
print "len(bot2x)",len(bot2x)

tck1x = interpolate.splrep(xnew,bot1x,s=0,k=5)
tck1y = interpolate.splrep(xnew,bot1y,s=0,k=5)
tck2x = interpolate.splrep(xnew,bot2x,s=0,k=5)
tck2y = interpolate.splrep(xnew,bot2y,s=0,k=5)

xnew1 = np.linspace(1,6,2*len(bot1x))

bot1x = interpolate.splev(xnew1,tck1x,der=0)
bot1y = interpolate.splev(xnew1,tck1y,der=0)
bot2x = interpolate.splev(xnew1,tck2x,der=0)
bot2y = interpolate.splev(xnew1,tck2y,der=0)

bot1x_dot = interpolate.splev(xnew1,tck1x,der=1)
bot1y_dot = interpolate.splev(xnew1,tck1y,der=1)
bot2x_dot = interpolate.splev(xnew1,tck2x,der=1)
bot2y_dot = interpolate.splev(xnew1,tck2y,der=1)

bot1_dot = sqrt((bot1x_dot)**2+(bot1y_dot)**2)
bot2_dot = sqrt((bot2x_dot)**2+(bot2y_dot)**2)

yaw1 = [math.atan2(bot1y_dot[i],bot1x_dot[i]) for i in range(len(bot1x_dot))]
yaw2 = [math.atan2(bot2y_dot[i],bot2x_dot[i]) for i in range(len(bot2x_dot))]
'''
Path tracking
'''

dt = float(format((xnew1[2]-xnew1[1]),'.2f'))  # [s] time difference
state = State(x=bot1x[0], y=bot1y[0], yaw=yaw1[0], v=0.0)
state2 = State(x=bot2x[0],y=bot2y[0], yaw=yaw2[0], v=0.0)

lastelement = len(bot1x) - 1
b1x = [0.0]
b1y = [0.0]
b2x = [0.0]
b2y = [0.0]
y1=[0.0] #yaw
y2=[0.0] #yaw

v1=[0.0]
v2=[0.0]
o1 = [0.0]
o2 = [0.0]
target_ind1, mind = calc_target_index(state, bot1x, bot1y,0)
target_ind2, mind = calc_target_index(state2, bot2x, bot2y,0)

rrt = RRT(start=[0, 0], goal=[10, 10],
              randAreax=[-2, 15],randAreay=[-2,15], obstacleList=circularObstacles)
    
error = 0.3
# while (lastelement  > target_ind1  or lastelement > target_ind2 ) and not (target_ind1==lastelement and target_ind2==lastelement) or :
while ((abs(bot1x[lastelement] - b1x[len(b1x)-1])>error) or (abs(bot1y[lastelement]-b1y[len(b1y)-1])>error) or (abs(bot2x[lastelement] - b2x[len(b2x)-1])>error) or (abs(bot2y[lastelement]-b2y[len(b2y)-1])>error)) :
    di, target_ind1 = stanley_control(state, bot1x, bot1y, yaw1, target_ind1)
    di2, target_ind2 = stanley_control(state2, bot2x, bot2y, yaw2, target_ind2)
    target_ind1 = min(target_ind1,target_ind2)
    target_ind2 = target_ind1
    ai = PIDControl(target_speed, state.v,target_ind1,state,bot1x,bot1y)
    state = update(state, ai, di)

    ai2 = PIDControl(target_speed, state2.v,target_ind2,state2,bot2x,bot2y)
    state2 = update(state2, ai2, di2)

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


    if show_animation:
        plt.cla()
        plt.plot(bot1x, bot1y, ".r", label="course")
        plt.plot(b1x, b1y, "-b", label="trajectory")
        plt.plot(bot1x[target_ind1], bot1y[target_ind1], "xg", label="target")
        plt.plot(bot2x, bot2y, ".r", label="course")
        plt.plot(b2x, b2y, "-b", label="trajectory")
        # plt.show()
        for obstacles in circularObstacles:
                    # ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='b'))
                    rrt.PlotCircle(obstacles[0], obstacles[1], obstacles[2])
        plt.plot(bot2x[target_ind2], bot2y[target_ind2], "xg", label="target")
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
print "v1"
print v1
max_vel1 = 0
max_vel2 = 0
for i in range(len(v1)) :
    if v1[i] > max_vel1 : 
        max_vel1 = v1[i]
    if v2[i] > max_vel2 :
        max_vel2 = v2[i]
    if v1[i]>0.3 :
        print v1[i]

print "max_vel1 :",max_vel1
print "max_vel2 :",max_vel2

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

xu1,yu1,phi1=np.array([0]),np.array([0]),np.array([0])
del_t = float(format((xnew1[2]-xnew1[1]),'.2f'))
for i in range(len(o1)) :
	phi1 = np.append(phi1,abs(phi1[len(phi1)-1]+(o1[i])*del_t))
	phi1[len(phi1)-1] = (math.atan2(math.tan(phi1[len(phi1)-1]),1))#+math.pi)
	xu1=np.append(xu1,xu1[len(xu1)-1]+v1[i]*math.cos(phi1[len(phi1)-1])*del_t)
	yu1=np.append(yu1,yu1[len(yu1)-1]+v1[i]*math.sin(phi1[len(phi1)-1])*del_t)

print "xu1"
print xu1
print "yu1"
print yu1
print "phi1"
print phi1
# plt.figure()
# plt.plot(xu1,yu1,'o',bot1x,bot1y,'--')


# print "del_t",del_t
# print "phi"
# print phi
# # print "xu"
# # for i in range(len(xu)):
# # 	print xu[i]
# # print xu
# print "bot1x"
# print bot1x
# # print "yu"
# # for i in range(len(yu)) :
# # 	print yu[i]
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
