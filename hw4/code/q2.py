import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def partial_fx(x):
    return 3*np.power(x,2) - 4*x

def partial_fy(y):
    return 3*np.power(y,2) + 6*y

def f_xy(x,y):
    return np.power(x,3) + np.power(y,3) - 2*np.power(x,2) + 3*np.power(y,2) - 8

def d(x,y):
    return (6*x-4)*(6*y+6)

def f_xx(x):
    return (6*x-4)

def plot_plane():
    fig,ax = plt.subplots()
    # ax = plt.axes(projection="3d")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    # ax.set_zlabel("Z")
    x = np.linspace(-5,5,100)
    y = np.linspace(-5,5,100)
    X,Y = np.meshgrid(x,y)
    Z = f_xy(X,Y)
    # ax.plot_surface(X,Y,Z,color='r')
    ax.contour(X,Y,Z)
    a = [1,4/3]
    b = [-1,0]
    # c = [f_xy(1,-1), f_xy(4/3,0)]
    ax.plot(a,b,color='b')
    # ax.arrow(1.0005, f_xy(1.0005,-0.9995), 1.0006 , f_xy(1.0006,-0.9994), head_width=0.05, head_length=0.1)
    plt.show()

if __name__ == "__main__":
    x = np.linspace(-3,3,100)
    y = np.linspace(-3,3,100)
    X,Y = np.meshgrid(x,y)
    z1 = partial_fx(x)
    z2 = partial_fy(y)

    # plt.plot(x,z2,color='b')
    # # plt.plot(x,z2,color='b')
    # plt.grid(True)
    # plt.title('Partial f_y')
    # plt.xlabel('y')
    # plt.ylabel('f_y(y)')
    # plt.show()
    plot_plane()