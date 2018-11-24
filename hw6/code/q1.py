import numpy as np
import sys
import matplotlib.pyplot as plt
import random
from util import Point, Stack
import functools
# from scipy.spatial import ConvexHull

p0 = ""

# return the second top element
def secondTop(S):
    p = S.top()
    S.pop()
    res = S.top()
    S.push(p)
    return res

# get the squared distance between two points
def squaredDistance(p1, p2):
    return (p1.getX() - p2.getX())**2 + (p1.getY() - p2.getY())**2

def orientation(p, q, r):
    val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - (q.getX() - p.getX()) * (r.getY() - q.getY())
    
    if val == 0:
        return 0 # colinear points
    if val > 0:
        return 1 # clockwise orientation
    return 2 # counter clockwise orientation

# compare function for calling in sorted for comparison against first point
def compare(p1, p2):
    orient = orientation(p0, p1, p2)
    if orient == 0:
        if squaredDistance(p0, p2) >= squaredDistance(p0, p1):
            return -1
        else:
            return 1

    if orient == 2:
        return -1
    return 1

# function that finds the convex hull of a set of points
def findConvexHull(points):
    ymin = points[0].getY()
    min_point = 0
    n = len(points)

    # find minimum y point and if y is same as ymin, find nearest x point
    for i in range(1, n):
        y = points[i].getY()
        x = points[i].getX()
        if (y < ymin) or (ymin == y and x < points[min_point].getX()):
            ymin = y
            min_point = i

    # swap min point and 1st point
    points[0], points[min_point] = points[min_point], points[0]

    global p0    
    p0 = points[0]    
    temp_points = points[1:]
    # created sorted list based on orientation
    sorted_list = sorted(temp_points, key=functools.cmp_to_key(compare))
    sorted_list.insert(0, p0)
    
    points = sorted_list
    
    # create a final array 
    m = 1
    for i in range(1,n):
        while (i < n-1 and (orientation(p0, points[i], points[i+1]) == 0)):
            i += 1
        
        points[m] = points[i]
        m += 1

    if m < 3:
        return 
    
    s = Stack()
    s.push(points[0])
    s.push(points[1])
    s.push(points[2])

    # check for orientation with temp hull and remove points
    for i in range(3, m):
        while (orientation (secondTop(s), s.top(), points[i]) != 2):
            s.pop()
        
        s.push(points[i])

    # return the stack
    return s

if __name__ == "__main__":
    p1 = Point(0,3)
    p2 = Point(1,1)
    p3 = Point(2,2)
    p4 = Point(4,4)
    p5 = Point(0,0)
    p6 = Point(1,2)
    p7 = Point(3,1)
    p8 = Point(3,3)
    p9 = Point(5,5)
    p10 = Point(1,5)
    p11 = Point(4.5,2)
    p12 = Point(4,0)
    points = [p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12]

    x_init = [i.getX() for i in points]
    y_init = [i.getY() for i in points]

    finalPoints = findConvexHull(points)

    x_final = []
    y_final = []
    
    while not finalPoints.is_empty():
        p = finalPoints.top()
        x_final.append(p.getX())
        y_final.append(p.getY())
        finalPoints.pop()

    plt.scatter(x_init, y_init, marker='o')
    plt.scatter(x_final, y_final, marker='^')
    plt.show()