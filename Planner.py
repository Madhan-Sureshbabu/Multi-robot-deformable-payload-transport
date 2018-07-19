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
bot_rad = 0.15
target_speed = 0.08
circularObstacles = [(-0.5,2.3,0.5),(-2.4,2.0,0.5),(-3.2,2.3,0.5)]#(-5, 2.5, 2),(-1,6,2.005),(-10,2,2),

# path_feasible = False 
safety_margin = 0.3 # gap between robots to avoid collision

"""
Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai(@Atsushi_twi)

"""


show_animation = True

fig, ax= plt.subplots()

ax.axis('equal')

class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randAreax,randAreay, expandDis=0.3, goalSampleRate=50, maxIter=1000):
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
        plt.axis([self.minrandx,self.maxrandx,self.minrandy,self.maxrandy])
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
            #   flag = 1
            #   return i
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


def main(Start,Goal):
    rrt = RRT(start=Start, goal=Goal,
              randAreax=[-3.3,-0.1],randAreay=[0,4], obstacleList=circularObstacles)
    path = rrt.Planning(animation=show_animation)
    path.reverse()
    # sine path
    # N = 150
    # n=np.linspace(0,149,50)
    # y_sin = (30 * np.sin(2*math.pi*n/N))
    # path = []
    # for i in range(len(n)):
    #   path.append([n[i],y_sin[i]])

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