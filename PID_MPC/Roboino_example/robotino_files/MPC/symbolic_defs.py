from casadi import *
from Configuration_robotino.config_m import conf_m
from Configuration_robotino.config_r import conf_r

class Sym_p_r(conf_r):
    def __init__(self):
        super().__init__()
        self.pr_pos=SX.sym('pos_p0',self.n_states)
        self.pr_x0=SX.sym('pr_x0',self.n_states)
        
        self.pr_u0=SX.sym('pr_u0',self.n_inputs)

        # Decision variable r*
        # self.pr_ref=SX.sym('p_ref',(self.N))
        
        # Parameters for the output to be followed
        self.xr_ref_desired=SX.sym('xr_desired',(self.n_states,self.N))

        self.pr_error=SX.sym('pr_error',3)
        #self.xkr = self.pr_x0#SX.sym('x_init',2)
        self.pr_vec=vertcat(self.pr_x0,veccat(self.xr_ref_desired))

class Sym_p_m(conf_m):
    def __init__(self):
        super().__init__()
        # self.pm_x0=SX.sym('pm_x0',self.n_states)

        # self.pm_u0=SX.sym('pm_u0',self.n_inputs,1)
        self.Q=100*np.eye(3)
        self.Q[1,1]=10
        self.Q[2,2]=10

        self.pm_error=SX.sym('p_w',self.delay_order,self.n_motors)

        self.rm=SX.sym('rm',self.n_motors,self.N)
        self.um=SX.sym('U',self.n_motors,self.N)

        self.omega=SX.sym('omega',self.n_inputs,self.N)
        self.currents=SX.sym('I',self.n_motors,self.N)



        self.eta=vertcat(self.currents,self.omega,)

        

        t=(self.rm-self.omega).T
        # t1=self.rm.T[:,0]-self.omega.T[:,0]
        # t2=self.rm.T[:,1]-self.omega.T[:,1]
        # t3=self.rm.T[:,2]-self.omega.T[:,2]

        ek=SX.zeros(self.N,(self.n_inputs*self.n_motors))

        ek[0,1:3]=self.pm_error[:,0]
        ek[0,4:6]=self.pm_error[:,1]
        ek[0,7:9]=self.pm_error[:,2]

        ek[1,2:3]=self.pm_error[0,0]

        ek[1,5:6]=self.pm_error[0,1]
        ek[1,8:9]=self.pm_error[0,2]



        ek[:,0]=t[:,0]
        ek[1:self.N,1]=t[0:self.N-1,0]
        ek[2:self.N,2]=t[0:self.N-2,0]


        ek[:,3]=t[:,1]
        ek[1:self.N,4]=t[0:self.N-1,1]
        ek[2:self.N,5]=t[0:self.N-2,1]

        ek[:,6]=t[:,2]
        ek[1:self.N,7]=t[0:self.N-1,2]
        ek[2:self.N,8]=t[0:self.N-2,2]
        self.e_k=ek.T

        self.pm=veccat(self.currents[:,0],self.omega[:,0],self.um[:,0],self.pm_error,)


        ################### # Decision variable r* ######################
        # r_m0=SX.sym('r_m0',(self.N))
        # r_m1=SX.sym('r_m1',(self.N))
        # r_m2=SX.sym('r_m2',(self.N))

        # # self.rm=vertcat(r_m0,r_m1,r_m2)

        # current=SX.sym('eta',3,self.N)

        # current[:,0]=self.pm_x0[0:3]


        # ek0_init=SX.sym('p_e0',2)        
        # ek1_init=SX.sym('p_e1',2)
        # ek2_init=SX.sym('p_e2',2)
        
        # omega0=SX.sym('omega0_k',self.N)
        # omega0[0]=self.pm_x0[3]
        # omega1=SX.sym('omega1_k',self.N)
        # omega1[0]=self.pm_x0[4]
        # omega2=SX.sym('omega2_k',self.N)
        # omega2[0]=self.pm_x0[5]
        # #self.current=SX.sym('i_k',3,self.N-1)

        # e01=SX.zeros(self.N,self.n_motors)
        # e01[0,1:3]=ek0_init
        # e01[1,2]=ek0_init[1]
        
        # e11=SX.zeros(self.N,self.n_motors)
        # e11[0,1:3]=ek1_init
        # e11[1,2]=ek1_init[1]

        # e21=SX.zeros(self.N,self.n_motors)
        # e21[0,1:3]=ek2_init
        # e21[1,2]=ek2_init[1]

        # t1=r_m0-omega0
        # e01[:,0]=t1
        # e01[1:self.N,1]=t1[0:self.N-1]#self.r_m0[0:self.N-1]-self.omega0[0:self.N-1]
        # e01[2:self.N,2]=t1[0:self.N-2]#self.r_m0[0:self.N-2]-self.omega0[0:self.N-2]
        # e_0=e01.T
        
        # t2=r_m1-omega1
        # e11[:,0]=t2
        # e11[1:self.N,1]=t2[0:self.N-1]#self.r_m1[0:self.N-1]-self.omega1[0:self.N-1]
        # e11[2:self.N,2]=t2[0:self.N-2]#self.r_m1[0:self.N-2]-self.omega1[0:self.N-2]
        # e_1=e11.T
        
        # t3=r_m2-omega2
        # e21[:,0]=t3  #self.r_m2-self.omega2
        # e21[1:self.N,1]=t3[0:self.N-1] #self.r_m2[0:self.N-1]-self.omega2[0:self.N-1]
        # e21[2:self.N,2]=t3[0:self.N-2]   #self.r_m2[0:self.N-2]-self.omega2[0:self.N-2]
        # e_2=e21.T
        # self.omega=vertcat(omega0.T,omega1.T,omega2.T)
        # self.eta=vertcat(current,self.omega)
        # self.e=vertcat(e_0,e_1,e_2)#-self.om
        # self.rm=reshape(vertcat(r_m0,r_m1,r_m2),self.N,self.n_motors).T
        # self.e_p=reshape(vertcat(ek0_init,ek1_init,ek2_init),2,3)
        # self.pm_vec=veccat(self.pm_u0,self.pm_x0,veccat(self.e_p))
