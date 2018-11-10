import numpy as np
import matplotlib.pyplot as plt
import math

if __name__ == "__main__":
    x = np.linspace(-math.pi/2, math.pi/2, 100)
    y = 0.5+np.sin(x)
    z = 0.5+0.724*x
    z2 = 0.5+0.774*x
    plt.plot(x,y,color='r')
    plt.plot(x,z,color='b', linestyle='dashed')
    plt.plot(x,z2,color='g', linestyle='dotted')
    plt.legend(['0.5+sinx', '0.5+0.724x', '0.5+0.774x'])
    plt.show()