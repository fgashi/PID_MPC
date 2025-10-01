import numpy as np
# import math
# x=np.arange(-1,1,0.1)
# print(x)


def ref_gen(start,stop,step):
    x_init= np.arange(start,stop,step)
    y_init=np.arange(start,stop,step)
    for j in range(len(x_init)):
        x_init[j] = x_init[j]/(stop)
        y_init[j] = 2*np.abs(x_init[j])*np.sqrt(1-np.abs(x_init[j]))
    return x_init,y_init