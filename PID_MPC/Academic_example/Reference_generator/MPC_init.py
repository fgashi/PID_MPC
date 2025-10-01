
from Model.model import sys_model#
import numpy as np
#import math


def mpc_init(x0,xt,N):
    
    return np.linspace(x0,xt,N)

# def build_init(u0,x_init,error,y_ref):
#     error[0]=-sys_model.sys_output(x_init)


# x0=5
# xt=-10
# N=20

# a=mpc_init(x0,xt,N)
# import matplotlib.pyplot as plt
# plt.plot(a)
# plt.show()