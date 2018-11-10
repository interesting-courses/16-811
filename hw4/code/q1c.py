import numpy as np
import math
import matplotlib.pyplot as plt

def f(x):
    y = np.round(x,3)
    if y == 1.0:
        return 0
    return np.power(y-1,1./3)

def f_derivative(y):
    a = 3*np.power(y,2)
    return 1/a

def _rk4(h,x,y):
    k1 = h*f_derivative(y)
    ystar = y+(k1/2)
    k2 = h*f_derivative(ystar)
    ystar = y+(k2/2)
    k3 = h*f_derivative(ystar)
    ystar = y+k3
    k4 = h*f_derivative(ystar)
    update = (k1+2*k2+2*k3+k4)/6
    return update

def rk4(h,N):
    y = np.zeros(N+1)
    x = np.zeros(N+1)
    func = np.zeros(N+1)
    error = np.zeros(N+1)
    x[N] = 2
    y[N] = 1
    func[N] = f(2)
    error[N] = y[N] - func[N]
    for i in range(N,0,-1):
        y[i-1] = y[i] + _rk4(h,x[i],y[i])
        x[i-1] = x[i]+h
        func[i-1] = f(x[i-1])
        error[i-1] = func[i-1] - y[i-1]

    # for i in range(x.shape[0]-1,-1,-1):
    #     print ("{0} & {1} & {2} & {3} \\\\".format(np.round(x[i],3), np.round(y[i],3), np.round(func[i],3), np.round(error[i],5)))
    print (np.sum(np.power(error[:N+1],2))/(N))
    return x,y,func,error

if __name__ == "__main__":
    h = -0.05
    N = 20
    x,y,func,error = rk4(h,N)
    plt.plot(x,y,color='b')
    plt.plot(x,func,color='r')
    plt.xlabel('Value of x')
    plt.ylabel('Value of function')
    plt.legend(['Estimated Function', 'Actual Function'])
    # plt.show()