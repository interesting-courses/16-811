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

def ab4(h,N):
    y = np.zeros(N+4)
    x = np.zeros(N+4)
    func = np.zeros(N+4)
    error = np.zeros(N+4)
    d = np.zeros(N+4)
    x[N+3],x[N+2],x[N+1],x[N] = 2.15,2.10,2.05,2.00
    y[N+3],y[N+2],y[N+1],y[N] = 1.04768955317165, 1.03228011545637, 1.01639635681485, 1.00
    d[N+3],d[N+2],d[N+1],d[N] = f_derivative(1.04768955317165), f_derivative(1.03228011545637), f_derivative(1.01639635681485), f_derivative(1.00)
    func[N+3],func[N+2],func[N+1],func[N] = f(2.15),f(2.1),f(2.05),f(2)
    error[N] = y[N] - func[N]
    for i in range(N,0,-1):
        update = (h/24) * (55*d[i] - 59*d[i+1] + 37*d[i+2] - 9*d[i+3])
        y[i-1] = y[i] + update
        d[i-1] = f_derivative(y[i-1])
        x[i-1] = x[i] + h
        func[i-1] = f(x[i-1])
        error[i-1] = func[i-1] - y[i-1]

    # for i in range(x.shape[0]-4,-1,-1):
    #     print ("{0} & {1} & {2} & {3} \\\\".format(np.round(x[i],3), np.round(y[i],3), np.round(func[i],3), np.round(error[i],5)))
    print (np.sum(np.power(error[:N+1],2))/(N))
    return x,y,func,error

if __name__ == "__main__":
    h = -0.05
    N = 20
    x,y,func,error = ab4(h,N)
    plt.plot(x,y,color='b')
    plt.plot(x,func,color='r')
    plt.xlabel('Value of x')
    plt.ylabel('Value of function')
    plt.legend(['Estimated Function', 'Actual Function'])
    # plt.show()