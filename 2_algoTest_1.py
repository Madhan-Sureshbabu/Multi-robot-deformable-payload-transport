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

pathFound, path = odm.plan(mf.runTime, mf.plannerType, mf.objectiveType, mf.fname, mf.bound, mf.start_pt, mf.goal_pt)

if pathFound:
	pathList = mf.getPathPointAsList(path)
#	print pathList
	pathList = np.array(pathList) 		# pathList is numpy array now
#	print "path is", pathList, "Obstacles are", mf.circularObstacles
else:
	print "Path not found..."

fig, ax= plt.subplots()
#fig1, ax1= plt.subplots()
ax.axis('equal')

# long_path = mf.getInterpolatedPath(pathList)
long_path = mf.curvefit(pathList,2*pathList.shape[0])


bot1_path = np.array([[0,0]])
bot1_dist = np.array([0.5-mf.bot_rad])
for i in range(1, long_path.shape[0]):
#for i in range(40, 51):
	final_dist = 10
	final_intersection = ()
	for obstacles in mf.circularObstacles:
		if (mf.circleOnLeft(long_path[i-1], long_path[i], obstacles)):
			ret, intersection = mf.intersectWithCircle(tuple(long_path[i]), tuple(long_path[i-1]), (obstacles[0], obstacles[1]), obstacles[2], np.pi/2)
			if ret == True:
				current_dist = mf.distance_between_points(intersection[0], intersection[1], long_path[i][0], long_path[i][1])
				if current_dist<final_dist:
					final_dist = current_dist
#					final_intersection = intersection

	current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
	bot1_theta = current_theta + math.pi/2

	if final_dist > 0.5:
		bot1_path = np.append(bot1_path, [[long_path[i][0] + (0.5-mf.bot_rad) * np.cos(bot1_theta), long_path[i][1] + (0.5-mf.bot_rad) * np.sin(bot1_theta)]], axis = 0)
		bot1_dist = np.append(bot1_dist, [(0.5-mf.bot_rad)], axis = 0)
	else:
		bot1_path = np.append(bot1_path, [[long_path[i][0] + (final_dist - mf.bot_rad) * np.cos(bot1_theta), long_path[i][1] + (final_dist - mf.bot_rad) * np.sin(bot1_theta)]], axis = 0)
		bot1_dist = np.append(bot1_dist, [(final_dist-mf.bot_rad)], axis = 0)

bot1_path_smooth = mf.curvefit(bot1_path,4*pathList.shape[0])

# bot1_dist_initial = np.copy(bot1_dist)
# bot1_dist = scipy.ndimage.minimum_filter1d(bot1_dist,size=25)
# bot1_dist = scipy_gaussian(bot1_dist,sigma=8)
# #bot1_dist = scipy.ndimage.median_filter(bot1_dist,size=10)
# #bot1_dist = scipy_gaussian(bot1_dist,sigma=3)
# #bot1_dist = scipy_gaussian(bot1_dist,sigma=10)


# bot1_path_smooth_fwd = np.array([[0,0]])
# bot1_forward_path = np.array([[0,0]])
# for i in range(1, long_path.shape[0]):
# #	ax.add_patch(plt.Circle((bot1_path_smooth[-1][0], bot1_path_smooth[-1][1]), mf.bot_rad, color='g'))
# 	current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
# 	bot1_theta = current_theta + math.pi/2
# 	next_pt =np.array([[long_path[i][0] + (bot1_dist[i]) * np.cos(bot1_theta), long_path[i][1] + (bot1_dist[i]) * np.sin(bot1_theta)]])
# 	a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
# 	b = [bot1_path_smooth_fwd[-1][0] - long_path[i-1][0] , bot1_path_smooth_fwd[-1][1] - long_path[i-1][1]]
# 	if np.cross(a,b) > 0:
# 		bot1_path_smooth_fwd = np.append(bot1_path_smooth_fwd, next_pt, axis = 0)
# 		bot1_forward_path = np.append(bot1_forward_path, next_pt, axis = 0)
# 	else:
# #		print "Pass"
# #		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
# 		bot1_forward_path = np.append(bot1_forward_path, np.array([[np.nan, np.nan]]), axis = 0)
			
		


# bot1_path_smooth_bck = np.array([[10,10]])
# for i in range(long_path.shape[0]-1, 0, -1):
# #	ax.add_patch(plt.Circle((bot1_path_smooth[-1][0], bot1_path_smooth[-1][1]), mf.bot_rad, color='g'))
# 	current_theta = math.atan2(float(long_path[i-1][1] - long_path[i][1]), float(long_path[i-1][0] - long_path[i][0]))
# 	bot1_theta = current_theta - math.pi/2
# 	next_pt =np.array([[long_path[i][0] + (bot1_dist[i]) * np.cos(bot1_theta), long_path[i][1] + (bot1_dist[i]) * np.sin(bot1_theta)]])
# 	a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
# 	b = [bot1_path_smooth_bck[-1][0] - long_path[i-1][0] , bot1_path_smooth_bck[-1][1] - long_path[i-1][1]]
# 	if np.cross(a,b) < 0:
# 		bot1_path_smooth_bck = np.append(bot1_path_smooth_bck, next_pt, axis = 0)
# #	else:
# #		print "Pass"
# #		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
# #		bot1_path_smooth_bck = np.append(bot1_path_smooth_bck, np.array([[np.nan, np.nan]]), axis = 0)
			
# #		bot1_path_smooth = np.append(bot1_path_smooth, [bot1_path_smooth[-1]], axis = 0)
	


# bot1_path_smooth = np.array([[0,0]])
# #print bot1_path_smooth_fwd.shape, bot1_path_smooth_bck[6]
# for  k in range(bot1_forward_path.shape[0]):
# #	print path_point.shape, bot1_path_smooth.shape
# 	if bot1_forward_path[k].tolist() in bot1_path_smooth_bck.tolist():
# 		bot1_path_smooth = np.append(bot1_path_smooth, [bot1_forward_path[k]], axis=0)
# 	else:
# 		bot1_path_smooth = np.append(bot1_path_smooth, [[np.nan, np.nan]], axis=0)

# print bot1_path_smooth.shape




##########################################################################################
#
#
#bot2_path = np.array([[0,0]])
#bot2_dist = np.array([0.5-mf.bot_rad])
#for i in range(1, long_path.shape[0]):
##for i in range(40, 51):
#	final_dist = 10
#	final_intersection = ()
#	for obstacles in mf.circularObstacles:
#		if (not mf.circleOnLeft(long_path[i-1], long_path[i], obstacles)):
#			ret, intersection = mf.intersectWithCircle(tuple(long_path[i]), tuple(long_path[i-1]), (obstacles[0], obstacles[1]), obstacles[2], -np.pi/2)
#			if ret == True:
#				current_dist = mf.distance_between_points(intersection[0], intersection[1], long_path[i][0], long_path[i][1])
#				if current_dist<final_dist:
#					final_dist = current_dist
##					final_intersection = intersection
#
#	current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
#        bot2_theta = current_theta - math.pi/2
#
#	if final_dist > 0.5:
#		bot2_path = np.append(bot2_path, [[long_path[i][0] + (0.5-mf.bot_rad) * np.cos(bot2_theta), long_path[i][1] + (0.5-mf.bot_rad) * np.sin(bot2_theta)]], axis = 0)
#		bot2_dist = np.append(bot2_dist, [(0.5-mf.bot_rad)], axis = 0)
#	else:
#		bot2_path = np.append(bot2_path, [[long_path[i][0] + (final_dist - mf.bot_rad) * np.cos(bot2_theta), long_path[i][1] + (final_dist - mf.bot_rad) * np.sin(bot2_theta)]], axis = 0)
#		bot2_dist = np.append(bot2_dist, [(final_dist-mf.bot_rad)], axis = 0)
#
#bot2_dist_initial = np.copy(bot2_dist)
#bot2_dist = scipy.ndimage.minimum_filter1d(bot2_dist,size=25)
#bot2_dist = scipy_gaussian(bot2_dist,sigma=8)
##bot2_dist = scipy.ndimage.median_filter(bot2_dist,size=10)
##bot2_dist = scipy_gaussian(bot2_dist,sigma=3)
##bot2_dist = scipy_gaussian(bot2_dist,sigma=10)
#
#
#bot2_path_smooth_fwd = np.array([[0,0]])
#bot2_forward_path = np.array([[0,0]])
#for i in range(1, long_path.shape[0]):
##	ax.add_patch(plt.Circle((bot2_path_smooth[-1][0], bot2_path_smooth[-1][1]), mf.bot_rad, color='g'))
#	current_theta = math.atan2(float(long_path[i][1] - long_path[i-1][1]), float(long_path[i][0] - long_path[i-1][0]))
#	bot2_theta = current_theta - math.pi/2
#	next_pt =np.array([[long_path[i][0] + (bot2_dist[i]) * np.cos(bot2_theta), long_path[i][1] + (bot2_dist[i]) * np.sin(bot2_theta)]])
#	a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
#	b = [bot2_path_smooth_fwd[-1][0] - long_path[i-1][0] , bot2_path_smooth_fwd[-1][1] - long_path[i-1][1]]
#	if np.cross(a,b) < 0:
#		bot2_path_smooth_fwd = np.append(bot2_path_smooth_fwd, next_pt, axis = 0)
#		bot2_forward_path = np.append(bot2_forward_path, next_pt, axis = 0)
#	else:
##		print "Pass"
##		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
#		bot2_forward_path = np.append(bot2_forward_path, np.array([[np.nan, np.nan]]), axis = 0)
#			
#		
#
#
#bot2_path_smooth_bck = np.array([[10,10]])
#for i in range(long_path.shape[0]-1, 0, -1):
##	ax.add_patch(plt.Circle((bot2_path_smooth[-1][0], bot2_path_smooth[-1][1]), mf.bot_rad, color='g'))
#	current_theta = math.atan2(float(long_path[i-1][1] - long_path[i][1]), float(long_path[i-1][0] - long_path[i][0]))
#	bot2_theta = current_theta + math.pi/2
#	next_pt =np.array([[long_path[i][0] + (bot2_dist[i]) * np.cos(bot2_theta), long_path[i][1] + (bot2_dist[i]) * np.sin(bot2_theta)]])
#	a = [next_pt[0][0] - long_path[i-1][0] , next_pt[0][1] - long_path[i-1][1]]
#	b = [bot2_path_smooth_bck[-1][0] - long_path[i-1][0] , bot2_path_smooth_bck[-1][1] - long_path[i-1][1]]
#	if np.cross(a,b) > 0:
#		bot2_path_smooth_bck = np.append(bot2_path_smooth_bck, next_pt, axis = 0)
##	else:
##		print "Pass"
##		ax.add_patch(plt.Circle((long_path[i][0], long_path[i][1]), 0.01, color='r'))
##		bot2_path_smooth_bck = np.append(bot2_path_smooth_bck, np.array([[np.nan, np.nan]]), axis = 0)
#			
##		bot2_path_smooth = np.append(bot2_path_smooth, [bot2_path_smooth[-1]], axis = 0)
#	
#
#
#bot2_path_smooth = np.array([[0,0]])
##print bot2_path_smooth_fwd.shape, bot2_path_smooth_bck[6]
#for  k in range(bot2_forward_path.shape[0]):
##	print path_point.shape, bot2_path_smooth.shape
#	if bot2_forward_path[k].tolist() in bot2_path_smooth_bck.tolist():
#		bot2_path_smooth = np.append(bot2_path_smooth, [bot2_forward_path[k]], axis=0)
#	else:
#		bot2_path_smooth = np.append(bot2_path_smooth, [[np.nan, np.nan]], axis=0)
#
#print bot2_path_smooth.shape
#
#
###########################################################################################

bot2_path_smooth = mf.right_bot_path_generator(long_path, (-math.pi/2))
print bot2_path_smooth
pathX, pathY = long_path.T
for i in range(len(bot1_path_smooth)):
	if  (np.isnan(bot1_path_smooth[i][0])) | (np.isnan(bot2_path_smooth[i][0])):
		pass
	else:
		plt.cla()
		for obstacles in mf.circularObstacles:
			ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='r'))
		ax.plot(pathX, pathY, 'b')
		ax.add_patch(plt.Circle((bot1_path_smooth[i][0], bot1_path_smooth[i][1]), mf.bot_rad, color='b'))
		ax.add_patch(plt.Circle((bot2_path_smooth[i][0], bot2_path_smooth[i][1]), mf.bot_rad, color='g'))
		ax.plot()
		plt.pause(0.001)
	

bot1x, bot1y = bot1_path_smooth.T
ax.plot(bot1x, bot1y, 'go-')

bot2x, bot2y = bot2_path_smooth.T
ax.plot(bot2x, bot2y, 'ro-')


#bot1x, bot1y = bot1_path_smooth.T
#ax.plot(bot1x, bot1y, 'go-')
#
#bot2x, bot2y = bot2_path_smooth.T
#ax.plot(bot2x, bot2y, 'ro-')
#
#
#pathX, pathY = long_path.T
#ax.plot(pathX, pathY, 'bo')
#bot1x, bot1y = bot1_path.T
#ax.plot(bot1x, bot1y, 'r-')
#
##bot1x, bot1y = bot1_path_smooth_fwd.T
##ax.plot(bot1x, bot1y, 'bo')
##plt.cla()
#ax1.plot(bot2_dist_initial, 'ro')
#ax1.plot(bot2_dist, 'go')
plt.show()
exit()























