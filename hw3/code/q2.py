import numpy as np
import math
import sys
import os
import matplotlib.pyplot as plt

def create_basis(m,n):
    basis = np.zeros((101,m+n+1))
    for i in range(0, 101):
        x = i/100
        for j in range (0,m+1):
            basis[i,j] = math.pow(x,j)

    for i in range(0,101):
        x = i/100
        for j in range(m+1,basis.shape[1]):
            basis[i,j] = math.sin((j-m)*math.pi*x)

    return basis

def get_coefficients(P,q):
    return np.matmul(np.linalg.pinv(P),q)

def get_error(basis, coeff, values):
    return np.linalg.norm(np.matmul(basis,coeff) - values)

def process_function(m,n,data):
    basis = create_basis(m,n)
    coeff = get_coefficients(basis, data)
    error = get_error(basis, coeff, data)
    return basis, coeff, error

if __name__ == "__main__":
    data = np.loadtxt('../data/problem2.txt')
    data = np.reshape(data, (101,1))
    linear_indices = 10
    trig_indices = 10

    error_threshold = 0.00001
    min_error = 0.001
    actual_indices = ()
    least_coeff_count = 20
    least_coeff = 0
    least_basis = []
    basis, coeff, error = process_function(1,10,data)

    for i in range(1, linear_indices):
        for j in range(1, trig_indices):
            basis, coeff, error = process_function(i,j,data)
            if error < min_error:
                z = np.round(coeff, 3)
                if np.count_nonzero(z) < least_coeff_count:
                    min_error = error
                    least_basis = basis
                    least_coeff = z
                    actual_indices = (i,j)
                

    print (least_coeff, min_error, actual_indices)
    reconstructed_func = np.matmul(least_basis, least_coeff)
    x = np.linspace(0,1,101)
    plt.plot(x, data, color = 'b')
    plt.plot(x, reconstructed_func, color = 'g', linestyle='dotted', marker='o')
    plt.show()