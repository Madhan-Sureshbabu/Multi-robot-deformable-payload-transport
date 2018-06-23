from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og
from math import sqrt
from sys import argv
import argparse
import myFunc as mf
import ompl_demo as odm
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
import numpy as np
from numpy import *
from scipy.optimize import least_squares

NUM_ROBOTS = 2
re_plan = 0

robot_global	= {}
VS_global 		= {}
VS_reference	= {}
bot_path		= {}
VS_path			= {}
robot_global	= {}
new_bot_position = {}
new_bot_position[0] = {}
new_bot_position[1] = {}


robot_global[0] = [1,4,1]
robot_global[1] = [1,2,1]

for i in range(NUM_ROBOTS) :
	VS_reference[i]	=	[2*i-1,0,1]

for i in range(NUM_ROBOTS) :
	bot_path[i] = np.array([[robot_global[i][0],robot_global[i][1]]])

VS_path[0] = np.array([[0,0]])
	
def function(x):
	global VS_reference,VS_global,robot_global
	A = [[math.cos(x[2]),-1*math.sin(x[2]),x[0]],[math.sin(x[2]),math.cos(x[2]),x[1]],[0,0,1]]
	cost_function = []
	cost_function = np.array(cost_function)

	for i in range(NUM_ROBOTS) :
		VS_global[i] = np.matmul(A,VS_reference[i])
		a = VS_global[i] - robot_global[i]
		cost_function = np.append(cost_function,a,axis=0)

	return cost_function 



guess = np.array([0,0,0])



while (abs(robot_global[i][0]-mf.goal_pt[0])>0.8 or abs(robot_global[i][1]-mf.goal_pt[1])>0.8) :
	### LOOP FROM HERE	
	
	# print "x_solution.x ", x_solution.x
	# print "x_solution.cost ", x_solution.cost
	# print "x_solution.optimality ", x_solution.optimality
	re_plan = 0
	x_solution = least_squares(function,guess)
	pathFound, path = odm.plan(mf.runTime, mf.plannerType, mf.objectiveType, mf.fname, mf.bound, (x_solution.x[0],x_solution.x[1]), mf.goal_pt)

	if pathFound:
		pathList = mf.getPathPointAsList(path)
		print pathList
		pathList = np.array(pathList) 
	else: 
		print "Path not found"

	fig, ax= plt.subplots()

	ax.axis('equal')

	long_path,vel_x,vel_y = mf.curvefit(pathList,2*pathList.shape[0])

	theta = math.atan2(vel_y[0],vel_x[0])

	A_new = [[math.cos(theta),-1*math.sin(theta),long_path[1][0]],[math.sin(theta),math.cos(theta),long_path[1][1]],[0,0,1]]
	print A_new

	for i in range(NUM_ROBOTS) :
		new_bot_position[i] = np.matmul(A_new , VS_reference[i])
		print "new_bot_position[i] :",new_bot_position[i]
		print i
		flag,obstacle = mf.myClearance(new_bot_position[i][0],new_bot_position[i][1])
		if flag== -1 :
			re_plan=1

	for i in range(NUM_ROBOTS):
		if re_plan == 0 :
			bot_path[i] = np.append(bot_path[i],[[new_bot_position[i][0],new_bot_position[i][1]]],axis=0)
			VS_path[0] = np.append(VS_path[0],[[long_path[1][0],long_path[1][1]]],axis=0)
			robot_global[i][0] = new_bot_position[i][0]
			robot_global[i][1] = new_bot_position[i][1]
		# else :
		# 	re_plan = 1
		# if flag == -1:
		# 	while (flag==-1):
		# 		result, VS_reference[i] = mf.intersection_point(tuple(new_bot_position[i][0],new_bot_position[i][1]),tuple(x_solution[0],x_solution[1]),obstacle)
		# 		new_bot_position[i] = np.matmul(A_new * VS_reference)
		# 		flag, obstacle = mf.myClearance(new_bot_position[i])

 
print bot_path[0]
print "------"
print bot_path[1]


for i in range(len(bot_path[0])):
	plt.cla()
	for obstacles in mf.circularObstacles:
		ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='r'))
	ax.plot(VS_path[0][i][0],VS_path[0][i][1] , 'b')
	ax.add_patch(plt.Circle((bot_path[0][i][0], bot_path[0][i][1]), mf.bot_rad, color='b'))
	ax.add_patch(plt.Circle((bot_path[1][i][0], bot_path[1][i][1]), mf.bot_rad, color='g'))
	ax.plot()
	plt.pause(0.001)


bot1x, bot1y = bot1_path_smooth.T
ax.plot(bot1x, bot1y, 'go-')

bot2x, bot2y = bot2_path_smooth.T
ax.plot(bot2x, bot2y, 'ro-')




