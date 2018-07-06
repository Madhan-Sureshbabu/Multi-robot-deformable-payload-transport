from math import sqrt
import numpy as np
import math
from scipy.ndimage.filters import gaussian_filter as scipy_gaussian
import scipy.ndimage
import matplotlib.pyplot as plt

# Set parameters for planner. Check ompl_demo.py for more available parameters
runTime = 3
plannerType = 'BFMTstar'#'BITstar' 'RRTstart'
objectiveType = 'PathLength'
fname= None
safety_margin = 1
# Give start and goal points
bound = (0.0, 10.0) #Basically the step size
start_pt = (0,0)
goal_pt = (10,10)
circularObstacles = [(6, 2.5, 2),(4,8,2)]#,(8,4,0.5)]#[(8,2,2), (3,8,2), (6.5, 6.0, 0.5), (9,7,1), (6.5, 7.5, 0.5) ,(8, 5.5, 0.5)]
bot_rad = 0.13
bot_point = {}
val2 = {}


def distance_between_points(x1,y1,x2,y2):
	return sqrt((x1-x2)**2 + (y1-y2)**2)

def myClearance(x, y):
	
	for i in range(len(circularObstacles)):
		obstacles = circularObstacles[i]
		obstacle_radius_with_margin = obstacles[2]+safety_margin
		val = sqrt((x-obstacles[0])**2 + (y-obstacles[1])**2) - obstacle_radius_with_margin - 2*bot_rad
		if val > 0:
			success = True
		else:
			success = False
			obs = obstacles
			break;
	
	# print success
	if (success): 
		return 1
	else:
		return -1


def getPathPointAsList(path):
	# print path
	string = [s.strip() for s in path.splitlines()]
	# print string
	pathList = []
	for i in string:
		if i != '':
			print i
			pathList.append(i.split(" "))
	# print pathList
	for j in pathList:
		j[0] = float(j[0])
		j[1] = float(j[1])
	# print pathList
	return pathList




def getInterpolatedPath(pathList):
	long_path = np.copy([pathList[0]])
	resolution = bot_rad/16
	for i in range(1,pathList.shape[0]):
		temp_path = np.copy([pathList[i-1]])
		theta = np.arctan2(pathList[i][1]-pathList[i-1][1], pathList[i][0]-pathList[i-1][0])    
		dist = distance_between_points(pathList[i][0], pathList[i][1], pathList[i-1][0], pathList[i-1][1])
		temp_dist = distance_between_points(temp_path[0][0], temp_path[0][1], pathList[i-1][0], pathList[i-1][1])
		
		while(temp_dist < dist-resolution):
		        temp_path = np.append(temp_path, [[temp_path[-1][0] + (resolution)*np.cos(theta), temp_path[-1][1] + (resolution)*np.sin(theta)]], axis = 0)
		        temp_dist = distance_between_points(temp_path[-1][0], temp_path[-1][1], temp_path[0][0], temp_path[0][1])
		
		long_path = np.append(long_path, temp_path, axis = 0)

	return long_path

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

def curvature(pathList,length,order = 3) :
	x = pathList[:,0]
	y = pathList[:,1]

	y_coeff = np.polyfit(x,y,order)
	func_y = np.poly1d(y_coeff)
	func_y_dot = func_y.deriv()
	func_y_ddot = func_y_dot.deriv()

	new_points = np.linspace(x[0],x[-1],length)

	new_y = func_y(new_points)
	new_y_dot = func_y_dot(new_points)
	new_y_ddot = func_y_ddot(new_points)

	R = ((1+(new_y_dot)**2)**1.5)/(abs(new_y_ddot)) 

	k = 1/R 
	# print k

	return k


# def intersection_point((x,y),(x0,y0),obstacle) : 
# 	m = math.atan2(float(y-y0)/float(x-x0))

# 	c = y - (m*x)
# 	p = obstacle[0]
# 	q = obstacle[1]
# 	r = obstacle[2]
# 	A = m**2 + 1
# 	B = 2*(m*c - m*q -p)
# 	C = q**2 - r**2 + p**2 - 2*c*q + c**2
# 	discriminant = B**2 - 4*A*C

#  if discriminant < 0:
#             return False, (0,0)
#     if discriminant == 0:
#             x_new = -(B/(2*A))
#             y_new = m*x_new + c
#             return True, (x_new, y_new)
#     if discriminant > 0:
#             x1 = ((-B) + math.sqrt(discriminant))/(2*A)
#             y1 = m*x1 + c
#             x2 = ((-B) - math.sqrt(discriminant))/(2*A)
#             y2 = m*x2 + c

#             if (((x1 - x)**2 + (y1 - y)**2) < ((x2 - x)**2 + (y2 - y)**2)):
#                     return True, (x1, y1)
#             else:
#                     return True, (x2, y2)




def intersectWithCircle((x, y), (x_prev, y_prev), (center_x, center_y), r, angle_in_formation): #angle_in_formation in radians
        if (x-x_prev) == 0:
                m = 0
        else:
                m = angle_in_formation + math.atan(float(y-y_prev)/float(x-x_prev))
        if m >= math.pi:
                m = m - math.pi
        m = math.tan(m)
        c = y - (m * x)
#       print "slope is" , m
#       print "c is", c
        p = center_x
        q = center_y
        A = m**2 + 1
        B = 2*(m*c - m*q -p)
        C = q**2 - r**2 + p**2 - 2*c*q + c**2

        discriminant = B**2 - 4*A*C
        if discriminant < 0:
                return False, (0,0)
        if discriminant == 0:
                x_new = -(B/(2*A))
                y_new = m*x_new + c
                return True, (x_new, y_new)
        if discriminant > 0:
                x1 = ((-B) + math.sqrt(discriminant))/(2*A)
                y1 = m*x1 + c
                x2 = ((-B) - math.sqrt(discriminant))/(2*A)
                y2 = m*x2 + c

                if (((x1 - x)**2 + (y1 - y)**2) < ((x2 - x)**2 + (y2 - y)**2)):
                        return True, (x1, y1)
                else:
                        return True, (x2, y2)



def circleOnLeft(origin, point, circle):
	a = np.array([point[0]-origin[0], point[1]-origin[1]])
	b = np.array([circle[0]-origin[0], circle[1]-origin[1]])
	c = np.cross(a,b)
	if c>0:
		return True
	else:
		return False




	#########################################################################################
	
def right_bot_path_generator(long_path, bot_angle):
	bot2_path = np.array([[0,0]])
	bot2_dist = np.array([0.5-bot_rad])
	for i in range(1, long_path.shape[0]):
	#for i in range(40, 51):
		final_dist = 10
		final_intersection = ()
		for obstacles in circularObstacles:
			if (not circleOnLeft(long_path[i-1], long_path[i], obstacles)):
				ret, intersection = intersectWithCircle(tuple(long_path[i]), tuple(long_path[i-1]), (obstacles[0], obstacles[1]), obstacles[2], bot_angle)
				if ret == True:
					current_dist = distance_between_points(intersection[0], intersection[1], long_path[i][0], long_path[i][1]) 
					if current_dist<final_dist:
						final_dist = current_dist
	#					final_intersection = intersection
	
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
		bot2_theta = current_theta + ( bot_angle)
	
		if final_dist > 0.5:
			bot2_path = np.append(bot2_path, [[long_path[i][0] + (0.5-bot_rad) * np.cos(bot2_theta), long_path[i][1] + (0.5-bot_rad) * np.sin(bot2_theta)]], axis = 0)
			bot2_dist = np.append(bot2_dist, [(0.5-bot_rad)], axis = 0)
		else:
			bot2_path = np.append(bot2_path, [[long_path[i][0] + (final_dist - bot_rad) * np.cos(bot2_theta), long_path[i][1] + (final_dist - bot_rad) * np.sin(bot2_theta)]], axis = 0)
			bot2_dist = np.append(bot2_dist, [(final_dist-bot_rad / abs(np.sin(bot_angle)))], axis = 0)
	
	bot2_dist_initial = np.copy(bot2_dist)
	
	bot2_path = curvefit(bot2_path,2*bot2_path.shape[0])

	return bot2_path
	# bot2_dist = scipy.ndimage.minimum_filter1d(bot2_dist,size=25)
	# bot2_dist = scipy_gaussian(bot2_dist,sigma=8)
	# #bot2_dist = scipy.ndimage.median_filter(bot2_dist,size=10)
	# #bot2_dist = scipy_gaussian(bot2_dist,sigma=3)
	# #bot2_dist = scipy_gaussian(bot2_dist,sigma=10)
	
	
	# bot2_path_smooth_fwd = np.array([[0,0]])
	# bot2_forward_path = np.array([[0,0]])
	# for i in range(1, long_path.shape[0]):
	# #	ax.add_patch(plt.Circle((bot2_path_smooth[-1][0], bot2_path_smooth[-1][1]), bot_rad, color='g'))
	# 	current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
	# 	bot2_theta = current_theta + (bot_angle)
	# 	next_pt =np.array([[long_path[i][0] + (bot2_dist[i]) * np.cos(bot2_theta), long_path[i][1] + (bot2_dist[i]) * np.sin(bot2_theta)]])
	# 	a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
	# 	b = [bot2_path_smooth_fwd[-1][0] - long_path[i-1][0] , bot2_path_smooth_fwd[-1][1] - long_path[i-1][1]]
	# 	if np.cross(a,b) < 0:
	# 		bot2_path_smooth_fwd = np.append(bot2_path_smooth_fwd, next_pt, axis = 0)
	# 		bot2_forward_path = np.append(bot2_forward_path, next_pt, axis = 0)
	# 	else:
	# #		print "Pass"
	# #		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
	# 		bot2_forward_path = np.append(bot2_forward_path, np.array([[np.nan, np.nan]]), axis = 0)
				
			
	
	
	# bot2_path_smooth_bck = np.array([[10,10]])
	# for i in range(long_path.shape[0]-1, 0, -1):
	# #	ax.add_patch(plt.Circle((bot2_path_smooth[-1][0], bot2_path_smooth[-1][1]), bot_rad, color='g'))
	# 	current_theta = math.atan2(float(long_path[i-1][1] - long_path[i][1]), float(long_path[i-1][0] - long_path[i][0]))
	# 	bot2_theta = current_theta + (np.pi + bot_angle)
	# 	next_pt =np.array([[long_path[i][0] + (bot2_dist[i]) * np.cos(bot2_theta), long_path[i][1] + (bot2_dist[i]) * np.sin(bot2_theta)]])
	# 	a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
	# 	b = [bot2_path_smooth_bck[-1][0] - long_path[i-1][0] , bot2_path_smooth_bck[-1][1] - long_path[i-1][1]]
	# 	if np.cross(a,b) > 0:
	# 		bot2_path_smooth_bck = np.append(bot2_path_smooth_bck, next_pt, axis = 0)
	# #	else:
	# #		print "Pass"
	# #		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
	# #		bot2_path_smooth_bck = np.append(bot2_path_smooth_bck, np.array([[np.nan, np.nan]]), axis = 0)
				
	# #		bot2_path_smooth = np.append(bot2_path_smooth, [bot2_path_smooth[-1]], axis = 0)
		
	
	
	# bot2_path_smooth = np.array([[0,0]])
	# #print bot2_path_smooth_fwd.shape, bot2_path_smooth_bck[6]
	# for  k in range(bot2_forward_path.shape[0]):
	# #	print path_point.shape, bot2_path_smooth.shape
	# 	if bot2_forward_path[k].tolist() in bot2_path_smooth_bck.tolist():
	# 		bot2_path_smooth = np.append(bot2_path_smooth, [bot2_forward_path[k]], axis=0)
	# 	else:
	# 		bot2_path_smooth = np.append(bot2_path_smooth, [[np.nan, np.nan]], axis=0)
	
	# return bot2_path_smooth
	
	
	##########################################################################################






def left_bot_path_generator(long_path, bot_angle):
	bot1_path = np.array([[0,0]])
	bot1_dist = np.array([0.5-bot_rad])
	for i in range(1, long_path.shape[0]):
	#for i in range(40, 51):
		final_dist = 10
		final_intersection = ()
		for obstacles in circularObstacles:
			if (circleOnLeft(long_path[i-1], long_path[i], obstacles)):
				ret, intersection = intersectWithCircle(tuple(long_path[i]), tuple(long_path[i-1]), (obstacles[0], obstacles[1]), obstacles[2], bot_angle)
				if ret == True:
					current_dist = distance_between_points(intersection[0], intersection[1], long_path[i][0], long_path[i][1]) 
					if current_dist<final_dist:
						final_dist = current_dist
	#					final_intersection = intersection
	
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
	        bot1_theta = current_theta + bot_angle
	
		if final_dist > 0.5:
			bot1_path = np.append(bot1_path, [[long_path[i][0] + (0.5-bot_rad) * np.cos(bot1_theta), long_path[i][1] + (0.5-bot_rad) * np.sin(bot1_theta)]], axis = 0)
			bot1_dist = np.append(bot1_dist, [(0.5-bot_rad)], axis = 0)
		else:
			bot1_path = np.append(bot1_path, [[long_path[i][0] + (final_dist - bot_rad) * np.cos(bot1_theta), long_path[i][1] + (final_dist - bot_rad) * np.sin(bot1_theta)]], axis = 0)
			bot1_dist = np.append(bot1_dist, [(final_dist-bot_rad / abs(np.sin(bot_angle)))], axis = 0)
	
	bot1_dist_initial = np.copy(bot1_dist)
	bot1_dist = scipy.ndimage.minimum_filter1d(bot1_dist,size=25)
	bot1_dist = scipy_gaussian(bot1_dist,sigma=8)
	#bot1_dist = scipy.ndimage.median_filter(bot1_dist,size=10)
	#bot1_dist = scipy_gaussian(bot1_dist,sigma=3)
	#bot1_dist = scipy_gaussian(bot1_dist,sigma=10)
	
	
	bot1_path_smooth_fwd = np.array([[0,0]])
	bot1_forward_path = np.array([[0,0]])
	for i in range(1, long_path.shape[0]):
	#	ax.add_patch(plt.Circle((bot1_path_smooth[-1][0], bot1_path_smooth[-1][1]), bot_rad, color='g'))
		current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
		bot1_theta = current_theta + bot_angle
		next_pt =np.array([[long_path[i][0] + (bot1_dist[i]) * np.cos(bot1_theta), long_path[i][1] + (bot1_dist[i]) * np.sin(bot1_theta)]])
		a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
		b = [bot1_path_smooth_fwd[-1][0] - long_path[i-1][0] , bot1_path_smooth_fwd[-1][1] - long_path[i-1][1]]
		if np.cross(a,b) > 0:
			bot1_path_smooth_fwd = np.append(bot1_path_smooth_fwd, next_pt, axis = 0)
			bot1_forward_path = np.append(bot1_forward_path, next_pt, axis = 0)
		else:
	#		print "Pass"
	#		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
			bot1_forward_path = np.append(bot1_forward_path, np.array([[np.nan, np.nan]]), axis = 0)
				
			
	
	bot1_path_smooth_bck = np.array([[10,10]])
	for i in range(long_path.shape[0]-1, 0, -1):
	#	ax.add_patch(plt.Circle((bot1_path_smooth[-1][0], bot1_path_smooth[-1][1]), bot_rad, color='g'))
		current_theta = math.atan2(float(long_path[i-1][1] - long_path[i][1]), float(long_path[i-1][0] - long_path[i][0]))
		bot1_theta = current_theta - (np.pi - bot_angle)
		next_pt =np.array([[long_path[i][0] + (bot1_dist[i]) * np.cos(bot1_theta), long_path[i][1] + (bot1_dist[i]) * np.sin(bot1_theta)]])
		a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
		b = [bot1_path_smooth_bck[-1][0] - long_path[i-1][0] , bot1_path_smooth_bck[-1][1] - long_path[i-1][1]]
		if np.cross(a,b) < 0:
			bot1_path_smooth_bck = np.append(bot1_path_smooth_bck, next_pt, axis = 0)
	#	else:
	#		print "Pass"
	#		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
	#		bot1_path_smooth_bck = np.append(bot1_path_smooth_bck, np.array([[np.nan, np.nan]]), axis = 0)
				
	#		bot1_path_smooth = np.append(bot1_path_smooth, [bot1_path_smooth[-1]], axis = 0)
		
	
	
	bot1_path_smooth = np.array([[0,0]])
	#print bot1_path_smooth_fwd.shape, bot1_path_smooth_bck[6]
	for  k in range(bot1_forward_path.shape[0]):
	#	print path_point.shape, bot1_path_smooth.shape
		if bot1_forward_path[k].tolist() in bot1_path_smooth_bck.tolist():
			bot1_path_smooth = np.append(bot1_path_smooth, [bot1_forward_path[k]], axis=0)
		else:
			bot1_path_smooth = np.append(bot1_path_smooth, [[np.nan, np.nan]], axis=0)
	
	return bot1_path_smooth




















