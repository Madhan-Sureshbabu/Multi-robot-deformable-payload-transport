# from ompl import util as ou
# from ompl import base as ob
# from ompl import geometric as og
# import cvxopt as cvx
from math import sqrt
from sys import argv
import argparse
import myFunc as mf
# import ompl_demo as odm
import matplotlib.pyplot as plt
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
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
import cubic_spline_planner
from scipy.interpolate import spalde
from scipy.interpolate import splev
from scipy.interpolate import splrep
import random
import copy
import sympy

"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai(@Atsushi_twi)

"""


show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 expandDis=0.5, goalSampleRate=20, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
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
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.__CollisionCheck(newNode, self.obstacleList):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation:
                self.DrawGraph(rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steer(self, rnd, nind):

        # expand tree
        nearestNode = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind
        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]
        #  print(goalinds)

        if len(goalinds) == 0:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)


    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):
        x, y = sympy.symbols("x y", real=True)
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            eq1 = sympy.Eq((x-ox)**2+(y-oy)**2,size**2)
            eq2 = sympy.Eq((x-node.x)**2+(y-node.y)**2,(2*mf.bot_rad)**2)
            sol = sympy.solve([eq1,eq2])
            if d <= size ** 2 or sol!=[]:
                print "collision"
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    # [x,y,size(radius)]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[5, 10],
              randArea=[-2, 15], obstacleList=circularObstacles)
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()



def velocity_of_VS_vertices(var):
	# var[0] = r1
	# var[1] = r2
	# var[2] = b1
	# var[3] = b2
	energy = 0
	for i in range(NUM_ROBOTS):
		A1 = np.array([[0,-k0[j],1],[k0[j],0,0],[0,0,0]])
		pos[i] = np.array([var[i]*math.cos(var[i+NUM_ROBOTS]),var[i]*math.sin(var[i+NUM_ROBOTS]),1])
		vel[i] = np.matmul(A1,pos[i])
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

# def intersection_calc(hom_matrix) :
# 	hom_matrix_inv = inv(hom_matrix)
# 	for r in range(len(circularObstacles)) :
# 		# obstacle[2] = obstacle[2] + safety_margin
# 		obstacle = circularObstacles[r]
# 		a = np.matmul(hom_matrix_inv,np.array([obstacle[0],obstacle[1],1]))
# 		obs_centre_x = a[0]
# 		obs_centre_y = a[1] 
# 		# print "obstacle[2]**2 - obs_centre_x**2",obstacle[2]**2 - obs_centre_x**2
# 		if obstacle[2]**2 - obs_centre_x**2 >= 0 :
# 			y_intersection1 = obs_centre_y - abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
# 			y_intersection2 = obs_centre_y + abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
# 			if obs_centre_y >= 0 :
# 				return y_intersection1
# 			else :
# 				return y_intersection2




def bounds_calc(hom_matrix,j):
	global path_feasible
	x, y = sympy.symbols("x y", real=True)
	# mf.safety()
	hom_matrix_inv = inv(hom_matrix)
	for r in range(len(circularObstacles)) :
		# obstacle[2] = obstacle[2] + safety_margin
		obstacle = circularObstacles[r]
		a = np.matmul(hom_matrix_inv,np.array([obstacle[0],obstacle[1],1]))
		obs_centre_x = a[0]
		obs_centre_y = a[1] 
		# print "obstacle[2]**2 - obs_centre_x**2",obstacle[2]**2 - obs_centre_x**2
		if obstacle[2]**2 - obs_centre_x**2 >= 0 :
			y_intersection1 = obs_centre_y - abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
			y_intersection2 = obs_centre_y + abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
			if obs_centre_y > 0 :
				if y_intersection1 - mf.bot_rad  < 0.5 :#and y_intersection1 - mf.bot_rad - safety_margin > 0:#2*mf.bot_rad:
					y_var = y_intersection1 - mf.bot_rad
					while (y_var > 0):
						eq1 = sympy.Eq((x-obs_centre_x)**2+(y-obs_centre_y)**2,obstacle[2]**2)
						eq2 = sympy.Eq((x)**2+(y-y_var)**2,(2*mf.bot_rad)**2)
						sol = sympy.solve([eq1,eq2])
						if sol==[] :
							break
						y_var = y_var - 0.1
					upper_bound[0][j] = y_var
					if upper_bound[0][j] <= lower_bound[0] :
						path_feasible = False
						upper_bound[0][j] = lower_bound[0] + 0.00001 #since upperbound > lower bound
					else :
						path_feasible = True
			elif obs_centre_y < 0 :
				if y_intersection2 + mf.bot_rad > -0.5 :#and y_intersection2 + mf.bot_rad + safety_margin < 0 :#-2*mf.bot_rad :
					y_var = y_intersection2 + mf.bot_rad
					while (y_var < 0) :
						eq1 = sympy.Eq((x-obs_centre_x)**2+(y-obs_centre_y)**2,obstacle[2]**2)
						eq2 = sympy.Eq((x)**2+(y-y_var)**2,(2*mf.bot_rad)**2)
						sol = sympy.solve([eq1,eq2])
						if sol==[] :
							break
						y_var = y_var + 0.1
					upper_bound[1][j] = y_var
					if upper_bound[1][j] >= -lower_bound[1] :
						upper_bound[1][j] = -lower_bound[1]-0.00001
						path_feasible = False
					else :
						path_feasible = True
	print "upper_bound[0]",upper_bound[0][j]
	print "upper_bound[1]",upper_bound[1][j]



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


for i in range(NUM_ROBOTS):
	mass[i] = 1
	vel_global[i] = {}
	bot_paths[i] = {}
	Bot_path[i]=np.array([[0,0]])
	lower_bound[i] = {}
	upper_bound[i] = {}
	print Bot_path[i]

circularObstacles = [(5, 2.5, 2),(2,6,2.005),(9,4,1)]

path_feasible = False 
safety_margin = 0.3 # gap between robots to avoid collision
pathList = main()


# def plan_path():
# global long_path,vel_x,vel_y,k0
# pathFound = True
# pathFound, path = odm.plan(mf.runTime, mf.plannerType, mf.objectiveType, mf.fname, mf.bound, mf.start_pt, mf.goal_pt)
# if pathFound:
# pathList = mf.getPathPointAsList(path)
# print pathList
pathList = np.array(pathList) 
print "len(pathList)",len(pathList)
long_path,vel_x,vel_y,k0 = mf.curvefit(pathList,pathList.shape[0])

vel_0 = sqrt((vel_x)**2+(vel_y)**2)
w_0 = vel_0 * k0
for i in range(NUM_ROBOTS):
	for j in range(len(long_path)) :
		upper_bound[i][j] = 0.5
		lower_bound[i] = mf.bot_rad - 0.001
	# return 1
# else: 
# 	print "Path not found"
	# return -1


guess = np.array([0.31001,.31001,0,-1*math.pi/2])
guess2 = np.array([0.001,0])
axis_guess = np.array([0,0,0])


# j = 3
# plan_path()

for j in range(1+len(long_path)) :
	if j != len(long_path) :
		axis_solution = least_squares(function,axis_guess)
		axis_theta = math.atan2(vel_y[j],vel_x[j])
		A = [[math.cos(axis_theta),-1*math.sin(axis_theta),axis_solution.x[0]],[math.sin(axis_theta),math.cos(axis_theta),axis_solution.x[1]],[0,0,1]]
		path_feasible = bounds_calc(A,j)
		# if path_feasible == False : 
		# 	break
		# path_feasible = True
		# print upper_bound[1][j]
		# r1 = cvx.Variable()
		# r2 = cvx.Variable()
		# b1 = cvx.Variable()
		# b2 = cvx.Variable()
		# constraints = [r1>=0.3,r1<=0.5,r2>=0.3,r2<=0.5,b1>=0,b1<=math.pi/2,b2>=0,b2<=math.pi/2]
		# obj = cvx.Minimize(velocity_of_VS_vertices)
		# prob = cvx.Problem(obj, constraints)
		# prob.solve()
		# print("status:", prob.status)
		# print("optimal value", prob.value)
		# print("optimal var", x.value, y.value)
		solution =  minimize(velocity_of_VS_vertices, guess,method='SLSQP', bounds = ((lower_bound[0],upper_bound[0][j]),(lower_bound[0],abs(upper_bound[1][j])),(0,math.pi),(0,math.pi)))
		# solution = least_squares(velocity_of_VS_vertices,guess,'3-point',bounds=([0.3,0.3,0,-math.pi], [ upper_bound[0][j],abs(upper_bound[1][j]) ,math.pi,0 ]))
		# solution = least_squares(velocity_of_VS_vertices2,guess2,'3-point',bounds=([2*mf.bot_rad, 0 ], [ upper_bound[1][j],math.pi ]))
		print "solution.x[0] ",solution.x[0]
		print "solution.x[1] ",solution.x[1]
		print "radius of curvature ",1/k0[j]
		print "position = ",long_path[j][0],long_path[j][1]
		print (1/k0[j])>solution.x[0]
		solution.x[3] = -1*solution.x[3]
		ICR_centre = np.matmul(A,np.array([0,1/k0[j],1]))

		circle.append(plt.Circle((ICR_centre[0],ICR_centre[1]),1/k0[j],color='b',fill=False))#np.append(circle,plt.Circle((ICR_centre[0],ICR_centre[1]),1/k0[j],color='b',fill=False),axis=0)
		# print "rho[0]",solution.x[0]
		# print "rho[1]",solution.x[1]
		# print "beta[0]",solution.x[2]
		# print "beta[1]",solution.x[3]
		# print "cost ",solution.cost
		# print "radius of curvature ",1/k0[j]
		# print "vel ",vel[1]
		# print "long_path ",long_path[j][0],long_path[j][1]
		# print "axis-theta ",axis_theta
		# print "axis",axis_solution.x
		# print "len(Bot_path[0])",len(Bot_path[0])
		# print "len(Bot_path[1])",len(Bot_path[1])

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
			# print "bot_paths[i][j] ",bot_paths[i][j]
			else :
				# print i,j
				# print bot_paths[i][j][1]
				# print bot_paths[i][j-1][1]
				# print bot_paths[i][j-2][1]
				m1 = ( bot_paths[i][j][1] - bot_paths[i][j-1][1] ) / (bot_paths[i][j][0] - bot_paths[i][j-1][0] )
				m2 = ( bot_paths[i][j-1][1] - bot_paths[i][j-2][1] ) / (bot_paths[i][j-1][0] - bot_paths[i][j-2][0] )
				# print "m1",m1
				# print "m2",m2
				# if m1*m2 >=-0.00009 :
				Bot_path[i] = np.append(Bot_path[i],np.array([[bot_paths[i][j][0],bot_paths[i][j][1]]]),axis=0)

					# else:
					# 	# print "Bot_path[i] b4",Bot_path[i]
					# 	# print "m1*m2",m1*m2
					# 	# print "deleting"
					# 	# print len(Bot_path[i])
					# 	Bot_path[i]=np.delete(Bot_path[i],len(Bot_path[i])-1,0)
					# 	# print len(Bot_path[i])
					# 	# Bot_path[i]=np.delete(Bot_path[i],len(Bot_path[i])-1,0)
					# 	for h in range(NUM_ROBOTS) :
					# 		if h!=i :
					# 			# print "deleting here too"
					# 			# print len(Bot_path[h])
					# 			Bot_path[h]=np.delete(Bot_path[h],len(Bot_path[h])-1,0)
					# 			Bot_path[h]=np.delete(Bot_path[h],len(Bot_path[h])-1,0)
								# print len(Bot_path[h])
								# Bot_path[h]=np.delete(Bot_path[h],len(Bot_path[h])-1,0)
						# Bot_path[i]=np.delete(Bot_path[i],len(Bot_path[i])-1,0)
						# Bot_path[i]=np.delete(Bot_path[i],len(Bot_path[i])-1,0)
						# Bot_path[i]=np.delete(Bot_path[i],len(Bot_path[i])-1,0)
						# print "Bot_path[i] after",Bot_path[i]
			# print "Bot_path[1] ",Bot_path[1][j]
		# print "vel_global ", vel_global

# print vel_global
# print np.array(bot_paths[0][3][0],bot_paths[0][3][1])
# Bot_path[0]=np.append(Bot_path[0],np.array())

fig, ax= plt.subplots()

ax.axis('equal')

# print bot_paths[0][3],bot_paths[0][]

# for i in range(1+len(long_path)) :
# 	if (i!=len(long_path)) and i!=0 :
		
# 		ax.add_artist(circle[i])
		# plt.cla()
		# # ax.add_patch(plt.Circle((long_path[i][0],long_path[i][1]),mf.bot_rad,color='b'))
		# ax.add_patch(plt.Circle((bot_paths[0][i][0],bot_paths[0][i][0]),mf.bot_rad,color='g'))
		# ax.add_patch(plt.Circle((bot_paths[1][i][0],bot_paths[1][i][1]),mf.bot_rad,color='r'))
		# ax.plot()
		# plt.pause(0.0.51)

		# bot1x[i], bot1y[i] = bot_paths[0][i][0],bot_paths[0][i][1]
		# bot2x[i], bot2y[i] = bot_paths[1][i][0],bot_paths[1][i][1]

		
# plt.cla()
# ax.plot()

for obstacles in circularObstacles:
			ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='b'))

# for h in range(NUM_ROBOTS) :
# 	Bot_path[h]=np.delete(Bot_path[h],len(Bot_path[h])-1,0)
# 	Bot_path[h]=np.delete(Bot_path[h],0,0)
	
# bx1 = Bot_path[0][0]
# by1 = Bot_path[0][1]
# bx2 = Bot_path[1][0]
# by2 = Bot_path[1][1]

# p = np.linspace(0, 20,len(Bot_path[0][0]), endpoint=True)
# Bot_path[0] = sorted(Bot_path[0],key=lambda p:p[0])
# Bot_path[1] = sorted(Bot_path[1],key=lambda p:p[0])
# Bot_path[0] = interp1d(Bot_path[0][0],Bot_path[0][1],kind='cubic',axis=0,copy=True,bounds_error=None,fill_value=None,assume_sorted=False)
# # Bot_path[0][1] = interp1d(p,Bot_path[0][1],kind='cubic',axis=0,copy=True,bounds_error=None,fill_value=None,assume_sorted=False)
# Bot_path[1] = interp1d(Bot_path[1][0],Bot_path[1][1],kind='cubic',axis=0,copy=True,bounds_error=None,fill_value=None,assume_sorted=False)
# Bot_path[1][1] = interp1d(p,Bot_path[1][1],kind='cubic',axis=0,copy=True,bounds_error=None,fill_value=None,assume_sorted=False)
Bot_path[0] = np.delete(Bot_path[0],0,0)
Bot_path[1] = np.delete(Bot_path[1],0,0)


bot1x,bot1y = Bot_path[0].T
bot2x,bot2y = Bot_path[1].T
xc,yc = long_path.T

xnew = np.linspace(0, 1, len(bot1x))
xnew2 = np.linspace(0,1,len(bot2x))

print len(xnew)
print len(xnew2)
bot1x = np.asarray(bot1x).squeeze()
bot1y = np.asarray(bot1y).squeeze()
bot2x = np.asarray(bot2x).squeeze()
bot2y = np.asarray(bot2y).squeeze()
# print xnew
# print bot1x
# print bot1x.shape==xnew.shape

# bot1x,bot1y,a,b,c = cubic_spline_planner.calc_spline_course(bot1x,bot1y,0.3)
# bot2x,bot2y,a,b,c = cubic_spline_planner.calc_spline_course(bot2x,bot2y,0.3)

# fox2 = cubic_spline_planner.calc_spline_course(xnew,bot2x)
# foy2 = cubic_spline_planner.calc_spline_course(xnew,bot2y)

fx1 = interp1d(xnew, bot1x,kind=9)#, k=4, s=0)#interp1d(xnew,bot1x)
fx2 = interp1d(xnew2, bot2x,kind=9)#, k=4, s=0)
fy1 = interp1d(xnew, bot1y,kind=9)#, k=4, s=0)
fy2 = interp1d(xnew2, bot2y,kind=9)#, k=4, s=0)




# tck1 = splrep(bot1x,bot1y,s=0)#, k=4, s=0)#interp1d(xnew,bot1x)
# tck2 = splrep(bot2x,bot2y,s=0)#, k=4, s=0)
# fy1 = splrep(xnew, bot1y,s=0)#, k=4, s=0)
# fy2 = splrep(xnew2, bot2y,s=0)#, k=4, s=0)


xnew1 = np.linspace(0,1,len(bot1x))

# ynew1 = splev(xnew, tck1, der=0)
# ynew2 = splev(xnew,tck2,,der=0)

bot1y = fy1(xnew1)
bot2y = fy2(xnew1)
bot1x = fx1(xnew1)
bot2x = fx2(xnew1)


# fy1_dot = splev(xnew,bot1x,der=1,ext=0)

# fy1_dot = fy1.derivative()# print bot1y
# fy2_dot = fy2.derivative()
# fx1_dot = fx1.derivative()
# fx2_dot = fx2.derivative()

# print fy1_dot(xnew1)
# print fx1_dot(xnew1)
# print fy2_dot(xnew1)
# print fx2_dot(xnew1)



# ax.plot(bot1x, bot1y, 'go-')

# path_x,path_y = long_path.T
# ax.plot(path_x,path_y,'bo')

# ax.plot(bot2x, bot2y, 'ro-')

# ax.plot(xc,yc,'bo-')
for i in range(len(Bot_path[0])):
	# print bot1y[i]
	ax.add_patch(plt.Circle((long_path[i][0],long_path[i][1]),2*mf.bot_rad,color='b'))
	ax.add_patch(plt.Circle((bot1x[i],bot1y[i]),mf.bot_rad,color='g'))
	ax.add_patch(plt.Circle((bot2x[i],bot2y[i]),mf.bot_rad,color='r'))
ax.plot()
# print bot2x,bot2y
plt.show()
exit()
