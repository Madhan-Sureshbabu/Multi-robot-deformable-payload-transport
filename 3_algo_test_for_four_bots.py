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

long_path = mf.getInterpolatedPath(pathList)


bot1_path_smooth = mf.left_bot_path_generator(long_path, (math.pi/4))
bot2_path_smooth = mf.right_bot_path_generator(long_path, (-math.pi/4))
bot3_path_smooth = mf.right_bot_path_generator(long_path, (-3*math.pi/4))
bot4_path_smooth = mf.left_bot_path_generator(long_path, (3*math.pi/4))
bot5_path_smooth = mf.left_bot_path_generator(long_path, (math.pi/2))
bot6_path_smooth = mf.right_bot_path_generator(long_path, (-math.pi/2))

pathX, pathY = long_path.T
for i in range(len(bot1_path_smooth)):
	if  (np.isnan(bot1_path_smooth[i][0])) | (np.isnan(bot2_path_smooth[i][0])| (np.isnan(bot3_path_smooth[i][0])) | (np.isnan(bot4_path_smooth[i][0]))):
#	if  (np.isnan(bot1_path_smooth[i][0])) | (np.isnan(bot2_path_smooth[i][0])):
		pass
	else:
		plt.cla()
		for obstacles in mf.circularObstacles:
			ax.add_patch(plt.Circle((obstacles[0], obstacles[1]), obstacles[2], color='r'))
		ax.plot(pathX, pathY, 'b')
		ax.add_patch(plt.Circle((bot1_path_smooth[i][0], bot1_path_smooth[i][1]), mf.bot_rad, color='b'))
		ax.add_patch(plt.Circle((bot2_path_smooth[i][0], bot2_path_smooth[i][1]), mf.bot_rad, color='g'))
		ax.add_patch(plt.Circle((bot3_path_smooth[i][0], bot3_path_smooth[i][1]), mf.bot_rad, color='g'))
		ax.add_patch(plt.Circle((bot4_path_smooth[i][0], bot4_path_smooth[i][1]), mf.bot_rad, color='g'))
		ax.add_patch(plt.Circle((bot5_path_smooth[i][0], bot5_path_smooth[i][1]), mf.bot_rad, color='g'))
		ax.add_patch(plt.Circle((bot6_path_smooth[i][0], bot6_path_smooth[i][1]), mf.bot_rad, color='g'))
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























