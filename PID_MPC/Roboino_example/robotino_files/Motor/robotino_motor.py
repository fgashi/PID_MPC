import math
import numpy as np
from Configuration_robotino.config_m import conf_m


# ###### First order dynamics only ###
class motor_model:
    def __init__(self):
        self.m1=m1()
        self.m2=m2()
        self.m3=m3()
        # self.R =22.375 
        # self.L = 5.1*1e-3#0.004
        # self.K = 0.003
        # self.Km=0.0475#*4
        # self.J =110 * 1e-7
        self.conf=conf_m()
        self.Ts=self.conf.Ts
        self.N=self.conf.N

        self.A=np.matrix([
            [ -self.m1.R_a/self.m1.L_a,                           0,                              0,    -self.m1.k_e/self.m1.L_a,                            0,                         0  ],
            [                        0,    -self.m2.R_a/self.m2.L_a,                              0,                         0,       -self.m2.k_e/self.m2.L_a,                         0  ],
            [                        0,                           0,       -self.m3.R_a/self.m3.L_a,                         0,                            0,    -self.m3.k_e/self.m3.L_a  ],#########
            [  self.m1.k_t/self.m1.J,                           0,                              0,      -self.m1.b/self.m1.J,                            0,                         0  ],
            [                        0,     self.m2.k_t/self.m2.J,                              0,                         0,         -self.m2.b/self.m2.J,                         0  ],
            [                        0,                           0,        self.m3.k_t/self.m3.J,                         0,                            0,      -self.m3.b/self.m3.J  ],#########
        ])
        
        
        self.B=np.matrix([
            [1./self.m1.L_a,                0,                 0],
            [0,                1./self.m2.L_a,                 0],
            [0,                             0,    1./self.m3.L_a],
            [0,                             0,                 0],
            [0,                             0,                 0],
            [0,                             0,                 0],
        ])


        # self.Ad=np.eye(6)+self.Ts*self.A+((self.Ts**2)/2)*(self.A*self.A.T)
        self.Ad=np.eye(6) + self.Ts*(self.A) + ((self.Ts**2)/2)*(np.linalg.matrix_power(self.A,2))
        self.Bd=self.Ts*((np.eye(6,dtype=float)+(self.Ts/2)*self.A)*(self.B))
        self.A_eta=self.Ad[0:3,0:3]
        self.B_eta=self.Bd[0:3,:]
        self.Kp=10.45#0.500
        self.Ki=2.5#0.49800
        self.Kd=1.430
        self.K1=self.Kp+(self.Ts/2)  * self.Ki +(1/self.Ts)  * self.Kd
        self.K2=-self.Kp+(self.Ts/2)  *self.Ki -(2/self.Ts)  * self.Kd
        self.K3=self.Kd/self.Ts
        # self.pid_mat=np.matrix([self.K1,self.K2,self.K3])
        # self.O_3=np.matrix(np.zeros((1,3)))
        # self.A_u=np.vstack((
        #     np.hstack((self.pid_mat,self.O_3,self.O_3)),
        #     np.hstack((self.pid_mat,self.O_3,self.O_3)),
        #     np.hstack((self.pid_mat,self.O_3,self.O_3))
        # ))
        self.A_u=np.matrix([
            [ self.K1,       self.K2,       self.K3,          0,          0,          0,          0,          0,          0  ],
            [        0,            0,             0,    self.K1,    self.K2,    self.K3,          0,          0,          0  ],
            [        0,            0,             0,          0,          0,          0,    self.K1,    self.K2,    self.K3  ]
        ])


class robotino_motor(motor_model):

    def __init__(self):
        super().__init__()

    
    def motor_dyn(self,X,U):
        return (self.Ad @ X)  + (self.Bd @ U)
    
    def pid_dyn(self,e,u_0):

        return self.A_u @ e + u_0
class m1:
    def __init__(self):
        self.k_p = 1.2543496 #V*s/rad (tunable)<<<<<<<<<<<<<<<<f
        self.J = 1.3262571e-06 #kg*m^2 (tunable)
        self.b = 2.56107e-05 #N*m*s/rad (tunable)
        self.T_0 = 0.0082467118 #N*m (tunable)
        self.L_a = 0.0089 #H (fixed)
        self.R_a = 5.95 #Ohm (fixed)
        self.k_t = 0.0514 #N*m/A (fixed)
        self.k_e = 0.0514 #V*s/rad (fixed)

class m2:
    def __init__(self):
        self.k_p = 1.2203558 #V*s/rad (tunable)
        self.J = 1.1671543e-06 #kg*m^2 (tunable)
        self.b = 1.8381794e-05 #N*m*s/rad (tunable)
        self.T_0 = 0.0077142278 #N*m (tunable)		
        self.L_a = 0.0089 #H (fixed)
        self.R_a = 5.95 #Ohm (fixed)
        self.k_t = 0.0514 #N*m/A (fixed)
        self.k_e = 0.0514 #V*s/rad (fixed)


class m3:
    def __init__(self):
        self.k_p=1.2231808  #V*s/rad (tunable)
        self.J = 1.2626015e-06   #kg*m^2 (tunable)
        self.b = 2.1569923e-05   # N*m*s/rad (tunable)
        self.T_0 = 0.0081146725  # N*m (tunable)
        self.L_a = 0.0089        # H (fixed)
        self.R_a = 5.95          #Ohm (fixed)
        self.k_t = 0.0514        # N*m/A (fixed)
        self.k_e = 0.0514        # V*s/rad (fixed)