from q4c import best_points
import sys
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def get_average_distance(points, normal, mean):
    normal = normal/np.linalg.norm(normal)
    dist = np.dot(points-mean, normal)
    sq_dist = np.dot(dist,dist)
    length = points.shape[0]
    final_dist = np.sqrt(sq_dist/length)
    return final_dist

def find_dominant_planes(points, num_iter=[500,500,500,500], threshold=[0.01,0.01,0.01,0.0,1]):
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.scatter(points[:,0], points[:,1], points[:,2], color='b')

    points1, normal1, mean1, new_points = best_points(points,num_iter[0],threshold[0])
    x = np.linspace(min(points1[:,0]),max(points1[:,0]),3)
    y = np.linspace(min(points1[:,1]),max(points1[:,1]),3)
    X,Y = np.meshgrid(x,y)
    Z = -(normal1[0]/normal1[2])*X -(normal1[1]/normal1[2])*Y + (np.dot(normal1, mean1)/normal1[2])
    ax.plot_surface(X,Y,Z,color='r')
    print ("The equation for the plane is {0}X+{1}Y+{2}Z+{3} = 0".format(str(normal1[0]), str(normal1[1]), str(normal1[2]), str(np.dot(normal1, mean1))))
    print ("Number of inliers is {0} and the average distance to the plane is {1}".format(str(points1.shape[0]), str(get_average_distance(points1, normal1, mean1))))

    points2, normal2, mean2, new_points = best_points(new_points,num_iter[1],threshold[1])
    x = np.linspace(min(points2[:,0]),max(points2[:,0]),3)
    y = np.linspace(min(points2[:,1]),max(points2[:,1]),3)
    X,Y = np.meshgrid(x,y)
    Z = -(normal2[0]/normal2[2])*X -(normal2[1]/normal2[2])*Y + (np.dot(normal2, mean2)/normal2[2])
    ax.plot_surface(X,Y,Z,color='g')
    print ("The equation for the plane is {0}X+{1}Y+{2}Z+{3} = 0".format(str(normal2[0]), str(normal2[1]), str(normal2[2]), str(np.dot(normal2, mean2))))
    print ("Number of inliers is {0} and the average distance to the plane is {1}".format(str(points2.shape[0]), str(get_average_distance(points2, normal2, mean2))))

    points3, normal3, mean3, new_points = best_points(new_points,num_iter[2],threshold[2])
    x = np.linspace(min(points3[:,0]),max(points3[:,0]),3)
    y = np.linspace(min(points3[:,1]),max(points3[:,1]),3)
    X,Y = np.meshgrid(x,y)
    Z = -(normal3[0]/normal3[2])*X -(normal3[1]/normal3[2])*Y + (np.dot(normal3, mean3)/normal3[2])
    ax.plot_surface(X,Y,Z,color='k')
    print ("The equation for the plane is {0}X+{1}Y+{2}Z+{3} = 0".format(str(normal3[0]), str(normal3[1]), str(normal3[2]), str(np.dot(normal3, mean3))))
    print ("Number of inliers is {0} and the average distance to the plane is {1}".format(str(points3.shape[0]), str(get_average_distance(points3, normal3, mean3))))

    points4, normal4, mean4, new_points = best_points(new_points,num_iter[3],threshold[3])
    x = np.linspace(min(points4[:,0]),max(points4[:,0]),3)
    y = np.linspace(min(points4[:,1]),max(points4[:,1]),3)
    X,Y = np.meshgrid(x,y)
    Z = -(normal4[0]/normal4[2])*X -(normal4[1]/normal4[2])*Y + (np.dot(normal4, mean4)/normal4[2])
    ax.plot_surface(X,Y,Z,color='y')
    print ("The equation for the plane is {0}X+{1}Y+{2}Z+{3} = 0".format(str(normal4[0]), str(normal4[1]), str(normal4[2]), str(np.dot(normal4, mean4))))
    print ("Number of inliers is {0} and the average distance to the plane is {1}".format(str(points4.shape[0]), str(get_average_distance(points4, normal4, mean4))))

    plt.show()

if __name__ == "__main__":
    filename = sys.argv[1]
    num_iter = [int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])]
    threshold = [float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]), float(sys.argv[9])]
    points = np.loadtxt(filename)
    find_dominant_planes(points, num_iter, threshold)