import numpy as np
from numpy import cos,sin, vstack, hstack, multiply
class sys_model:
    def __init__(self):
        self.sys_param=sys_param()


    def sys_sim(self,x0,u0):
       self.x=self.sys_param.Ad@x0+self.sys_param.Bd*u0
    #    self.x=x
       return self.x
    def sys_output(self,x0):
       return self.sys_param.Cd@x0
          
    def PID_sim(self,Ts,ev,ep,ep_1,u0):
       K1=self.sys_param.param.Kp+(Ts/2)  * self.sys_param.Ki +(1/Ts)  * self.sys_param.Kd
       K2=-self.sys_param.Kp+(Ts/2)  *self.sys_param.Ki -(2/Ts)  * self.sys_param.Kd
       self.u=u0+K1*(ev) + (K2 * (ep )) + (1/Ts)*(self.sys_param.Kd* (ep_1))
       return self.u


class sys_param:
  def __init__(self):
    self.a0=0.0453
    self.a1=0.0409512
    self.b1=-1.724
    self.b2=0.74119
    self.Ad=np.matrix([[0,1],[-self.b2,-self.b1]])
    self.Bd=np.matrix([[0],[1]])
    self.Cd=np.matrix([self.a1,self.a0])
    self.Kp=10.4500
    self.Ki=10#0.49800
    self.Kd=10.0#45