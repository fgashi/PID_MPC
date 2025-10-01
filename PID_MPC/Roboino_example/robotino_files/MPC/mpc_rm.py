import sys
from casadi import * 
# from Parameters_robotino.parameters_robotino import *
from Configuration_robotino.config_m import *
from Configuration_robotino.config_r import *
from Constraints_robotino.constraints_r import constraints_r
from Constraints_robotino.constraints_m import constraints_m
from MPC.symbolic_defs import *
from Motor.robotino_motor import robotino_motor
from Robot_model.R_model import Robotino_
from time import time


class mpc_rm:
    def __init__(self):
        # super().__init__()
        self.constraints_r=constraints_r()
        self.constraints_m=constraints_m()
        self.sym_r=Sym_p_r()
        self.sym_m=Sym_p_m()
        self.motor_model=robotino_motor()
        self.R_model=Robotino_()
        self.args={}
        self.args['lbx']  =  self.constraints_r.lbz_r  + self.constraints_m.lbz_m
        self.args['ubx']  =  self.constraints_r.ubz_r  + self.constraints_m.ubz_m
        self.args['lbg']  =  self.constraints_r.g_lb_r + self.constraints_m.g_lb_m
        self.args['ubg']  =  self.constraints_r.g_ub_r + self.constraints_m.g_ub_m
        # self.N_m=self.constraints_m.N
        # self.N_r=self.constraints_r.N
        self.n_states_r=self.constraints_r.n_states
        self.n_states_m=self.constraints_m.n_states
        # self.Ts=self.sym_r.Ts
        # self.N=self.sym_r.N
        self.opts={'ipopt.linear_solver':'ma27','ipopt.warm_start_init_point':'yes' }
        self.mpc=self.create_mpc_rm()



    def create_mpc_rm(self):

        xkr=self.sym_r.pr_x0
        xr_ref_desired=self.sym_r.xr_ref_desired

        # pkr=self.sym_r.pr_pos
        # pm_error=self.sym_m.pm_error
        self.currents=self.sym_m.currents
        omega=self.sym_m.omega
        rm=self.sym_m.rm
        um=self.sym_m.um
        eta=self.sym_m.eta
        e_k=self.sym_m.e_k
        pm=self.sym_m.pm
        # e0_k=self.sym_m.e_0
        # e1_k=self.sym_m.e_1
        # e3_k=self.sym_m.e_2
        x=[];
        u=[];
        # eta_=[];
        
        eta_acc=[]
        g1=[];
        g2=[];
        g3=[];
        
        J = 0;
    # # build the problem in here #
        for k in range(self.sym_m.N-1):
            

            #Robot subsystem

            xkr1=SX.sym('x_' + str(k+1), self.sym_r.n_states)
            
            g1          +=[-xkr1+self.R_model.runga_kutta(xkr,eta[3:6,k],self.sym_r.Ts)]
            x           +=[xkr1]

            #Motor subsystem
            
            g2        +=[-eta[:,k+1]+self.motor_model.motor_dyn(eta[:,k],um[:,k+1])]
            
            eta_acc       +=[ eta[:,k+1] ]


            u           +=[ um[:,k+1] ]
            g3        +=[ -um[:,k+1] + self.motor_model.pid_dyn(e_k[:,k],um[:,k]) ]
            xkr=xkr1
            e0= xkr1-xr_ref_desired[:,k+1]
            J= J + e0.T @ self.sym_m.Q @ e0
            
            
        self.z=x+eta_acc+u+[veccat(rm)]      
        self.g=g1+g2+g3
        pvec=veccat(self.sym_r.pr_x0,xr_ref_desired,pm)
        self.nlp_prob={'f':J,'x':veccat(*self.z),'g':veccat(*self.g),'p':pvec} 
        return nlpsol('solver','ipopt',self.nlp_prob,self.opts)
        # return qpsol('nlpsol','qrqp',self.nlp_prob)#,self.opts)







    def solve_mpc_rm(self,z0,p0):
        # time1=time()
        self.a=self.mpc(x0=z0,p=p0,lbx=self.args['lbx'],ubx=self.args['ubx'],lbg=self.args['lbg'],ubg=self.args['ubg'])
        # self.time2=time()-time1
        # print(self.mpc.stats()['t_wall_total'])
        # print('solving time: \n ',self.time2)
        return self.a['x'].full().flatten()
    

    def extract_states(self):
        pass