import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import sys

def get_normal_to_plane(points):
    imh, imw = points.shape
    mean = np.average(points, axis=0)
    R = points - mean
    w,v = np.linalg.eig(np.matmul(R.T,R))
    idx = np.argmin(w)
    normal = v[:,idx]
    return mean, normal

def plot_plane(points, normal, mean):
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.scatter(points[:,0], points[:,1], points[:,2], color='b')
    x = np.linspace(min(points[:,0]),max(points[:,0]),3)
    y = np.linspace(min(points[:,1]),max(points[:,1]),3)
    X,Y = np.meshgrid(x,y)
    Z = -(normal[0]/normal[2])*X -(normal[1]/normal[2])*Y + (np.dot(normal, mean)/normal[2])
    ax.plot_wireframe(X,Y,Z,color='r')
    plt.show()

def get_average_distanace(points, normal, mean):
    normal = normal/np.linalg.norm(normal)
    return np.mean(abs(np.dot(points-mean, normal)))

if __name__ == "__main__":
    filename = sys.argv[1]
    points = np.loadtxt(filename)
    mean, normal = get_normal_to_plane(points)
    plot_plane(points, normal, mean)
    avg_distance = get_average_distanace(points, normal, mean)
    print (normal, avg_distance, np.dot(normal,mean))