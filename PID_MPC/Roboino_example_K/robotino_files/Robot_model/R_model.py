# from operator import inv
# from sys import path
# path.append(r"<yourpath>/casadi-py27-v3.5.5")
# from casadi import *
# from casadi.tools import *
from casadi import *
from casadi.tools import *  # for dotdraw
import numpy as np
from numpy import cos,sin #vstack, hstack, multiply

class Robotino_:
  
  def __init__(self):

    self.param=robot_param()
    
  def f(self,x,omega):
    
    self.theta=x[2,0]#*np.pi/180
    
    
    self.R_z= np.matrix([[np.cos(self.theta) , -np.sin(self.theta), 0],
                      [np.sin(self.theta), np.cos(self.theta), 0],
                      [0,0,1]])
    # return ((self.param.rot_to_m) * self.R_z.dot(self.param.t_phi)) @ omega#*((2 * 3.142) / 60 )
    return self.R_z.dot(self.param.T_phi) @ omega
  def runga_kutta(self, X, U, Ts):
   
    
    self.Ts=Ts
    #for j in range(self.param.M):
    k1 = self.f(X, U)
    k2 = self.f(X + (Ts/2) * k1, U)
    k3 = self.f(X + (Ts/2 )* k2, U)
    k4 = self.f(X + (Ts) * k3, U)
    X=X+(self.Ts/6)*(k1 +2*k2 +2*k3 +k4)
    # X=X+(self.Ts)*k1
    self.X=X
    return self.X 



class robot_param:
  def __init__(self):
    self.M=4
    self.r = 0.040
    self.L=0.135
    self.gear=1.0/16
    self.alpha1=np.deg2rad(0)
    self.alpha2=np.deg2rad(120)
    self.alpha3=np.deg2rad(240)
    self.rot_to_m=(((2 * np.pi)*self.r) /(16* 60 ))
    self.T_phi=(self.r*self.gear)*np.matrix([[-1/np.sqrt(3), 0, 1/np.sqrt(3)],
                   [1/3,-2/3,1/3],
                   [1/(3*self.L),1/(3*self.L),1/(3*self.L)]])
    
    # self.t_phi=(self.r/16)*np.linalg.inv(np.matrix([[- np.sin(self.alpha1), np.cos(self.alpha1), self.L],
    #                                          [- np.sin(self.alpha2), np.cos(self.alpha2), self.L],
    #                                          [- np.sin(self.alpha3), np.cos(self.alpha3), self.L]
    #  ]))
    
    ###########print('here')
    # self.t_phi=1/16*np.matrix([[- self.r/np.sqrt(3),               0,   self.r/np.sqrt(3)],
    #                       [           self.r/3,   -(2*self.r)/3,         self.r/3,  ],      
    #                       [           self.r/(3*self.L),   -self.r/(3*self.L),         self.r/(3*self.L),  ]
    #  ])
    

    # self.t_phi=np.linalg.inv(np.matrix([
    #                            [             0,                  1,            self.L],
    #                            [       (-np.sqrt(3))/2,    -(1.0)/2,            self.L ],      
    #                            [       (np.sqrt(3))/2,   -(1.0)/2,              self.L]
                              #  ]))
    print('here')
      



# test=Robotino()
# x1=np.ones((3,1))
# w1=np.zeros((3,1))
# am=test.f_kin(x1,w1)
# print('here')