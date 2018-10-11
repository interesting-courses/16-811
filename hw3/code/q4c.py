import numpy as np
from q4 import plot_plane, get_average_distanace

def least_squares_fit(points):
    imh, imw = points.shape
    mean = np.average(points, axis=0)
    R = points - mean
    w,v = np.linalg.eig(np.matmul(R.T,R))
    idx = np.argmin(w)
    normal = v[:,idx]
    return normal, mean

def best_points(points, iterations = 2000, threshold = 0.003):
    """
        Takes in a set of points, performs ransac and finds the inliers
        smallest_num -> min number of points to form a plane
        iterations -> number of iterations to run ransac
        threshold -> error thresholding
        num_near -> number of nearby points required in the ransac
    """
    
    length_points = points.shape[0]
    maxInliers = -1
    bestInliers = np.zeros((1,1))
    for i in range(iterations):
        idx = np.random.permutation(length_points)
        sample_in = idx[0:3]
        p_in = points[sample_in,:]
        normal, mean = least_squares_fit(p_in)

        dist = abs(np.dot(points-mean, normal))
        inliers = dist < threshold
        numInliers = sum(inliers)
        
        if numInliers > maxInliers:
            maxInliers = numInliers
            bestInliers = inliers

    p_best = points[bestInliers,:]
    p_rest = points[~bestInliers,:]
    n_best, m_best = least_squares_fit(p_best)
    return p_best, n_best, m_best, p_rest

if __name__ == "__main__":
    points = np.loadtxt('cluttered_table.txt')
    final_points, final_normal, final_mean, p_rest = best_points(points)
    plot_plane(points, final_normal, final_mean)
    avg_distance = get_average_distanace(points, final_normal, final_mean)
    print (avg_distance, final_normal, np.dot(final_normal, final_mean))