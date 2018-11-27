import numpy as np
from matplotlib import pyplot as plt
import sys

def findConvexHull(points):
    points = np.array(points, dtype=np.float32)
    if points.shape[0] < 2:
        print ("Minimum 3 points required")
        sys.exit(1)

    points = points[np.lexsort(np.rot90(points))]

    lower, upper = [], []
    N = points.shape[0]
    for i in range(0,N):
        while len(lower)>=2 and -1*np.cross((points[i]-lower[-2]), (lower[-1]-lower[-2])) <= 0:
            lower.pop()
        lower.append(points[i])

    for j in range(N-1, -1,-1):
        while len(upper)>=2 and -1*np.cross((points[j]-upper[-2]), (upper[-1]-upper[-2])) <= 0:
            upper.pop()

        upper.append(points[j])

    hull = np.array(lower[:-1]+upper[:-1])
    return hull

if __name__ == "__main__":
    points = np.random.rand(500, 2)
    hull = findConvexHull(points)
    plt.plot(points[:,0],points[:,1], 'bo')
    plt.plot(hull[:,0], hull[:,1], 'r-')
    plt.show()