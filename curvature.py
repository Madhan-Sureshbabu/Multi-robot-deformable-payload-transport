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
from numpy import linalg as LA
from numpy.linalg import inv

def velocity_of_VS_vertices(var):
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


def bounds_calc(hom_matrix,j):
	hom_matrix_inv = inv(hom_matrix)
	for obstacle in mf.circularObstacles :
		a = np.matmul(hom_matrix_inv,np.array([obstacle[0],obstacle[1],1]))
		obs_centre_x = a[0]
		obs_centre_y = a[1] 
		# print "obstacle[2]**2 - obs_centre_x**2",obstacle[2]**2 - obs_centre_x**2
		if obstacle[2]**2 - obs_centre_x**2 > 0 :
			y_intersection = obs_centre_y - abs(sqrt(obstacle[2]**2 - obs_centre_x**2))
			if y_intersection > mf.bot_rad :
				upper_bound[1][j] = 1.5#y_intersection - mf.bot_rad
	return upper_bound[1][j]
			
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

for i in range(NUM_ROBOTS):
	mass[i] = 1
	vel_global[i] = {}
	bot_paths[i] = {}
	Bot_path[i]=np.array([[0,0]])
	lower_bound[i] = {}
	upper_bound[i] = {}

pathFound, path = odm.plan(mf.runTime, mf.plannerType, mf.objectiveType, mf.fname, mf.bound, mf.start_pt, mf.goal_pt)

if pathFound:
	pathList = mf.getPathPointAsList(path)
	# print pathList
	pathList = np.array(pathList) 
else: 
	print "Path not found"

long_path,vel_x,vel_y,k0 = mf.curvefit(pathList,pathList.shape[0])

for j in range(len(long_path)) :
	upper_bound[1][j] = 1.5

vel_0 = sqrt((vel_x)**2+(vel_y)**2)
w_0 = vel_0 * k0

guess = np.array([1,1,0,math.pi/2])
guess2 = np.array([0,0])
axis_guess = np.array([0,0,0])



# j = 3

for j in range(1+len(long_path)) :

	if j != len(long_path) :
		axis_solution = least_squares(function,axis_guess)
		axis_theta = math.atan2(vel_y[j],vel_x[j])
		A = [[math.cos(axis_theta),-1*math.sin(axis_theta),axis_solution.x[0]],[math.sin(axis_theta),math.cos(axis_theta),axis_solution.x[1]],[0,0,1]]
		bounds_calc(A,j)
		print upper_bound[1][j]
		# solution = least_squares(velocity_of_VS_vertices,guess,'3-point',bounds=([0.5, 0.5,0,math.pi/3], [ 1.5,1.5 ,math.pi/3,math.pi ]))
		solution = least_squares(velocity_of_VS_vertices2,guess2,'3-point',bounds=([0, 0 ], [ 1.5,math.pi ]))
		print solution.x[1]
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


		for i in range(NUM_ROBOTS) :
			A_vel = np.array([[math.cos(axis_theta),-1*math.sin(axis_theta),0],[math.sin(axis_theta),math.cos(axis_theta),0],[0,0,1]])
			vel_global[i][j] = np.matmul(A_vel,vel[i])
			if i==0 :
				bot_paths[i][j] = np.matmul(A,np.array([0.1*math.cos(0),0.1*math.sin(0),1]))
				Bot_path[i] = np.append(Bot_path[i],np.array([[bot_paths[i][j][0],bot_paths[i][j][1]]]),axis=0)
			else :
				bot_paths[i][j] = np.matmul(A,np.array([solution.x[i-1]*math.cos(solution.x[i]),solution.x[i-1]*math.sin(solution.x[i]),1]))
				Bot_path[i] = np.append(Bot_path[i],np.array([[bot_paths[i][j][0],bot_paths[i][j][1]]]),axis=0)
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
		# plt.pause(0.101)

		# bot1x[i], bot1y[i] = bot_paths[0][i][0],bot_paths[0][i][1]
		# bot2x[i], bot2y[i] = bot_paths[1][i][0],bot_paths[1][i][1]

		
# plt.cla()
# ax.plot()

bot1x,bot1y = Bot_path[0].T
ax.plot(bot1x, bot1y, 'go-')

# path_x,path_y = long_path.T
# ax.plot(path_x,path_y,'bo')

bot2x,bot2y = Bot_path[1].T
ax.plot(bot2x, bot2y, 'ro-')

print bot2x,bot2y
plt.show()
exit()
