
from casadi import *
from casadi.tools import *  # for dotdraw
import numpy as np
# import sys
# sys.path.append('/home/fatos/cpp_ws/publications/isr_robotics/reference_generation/Parameters')
# # from ..Parameters.parameters import *
from Parameters.parameters import parameters
from Model.model import sys_model
from Configuration import configuration
from Constraints.constraints import constr


class MPC:
    def __init__(self):
        self.parameters=parameters()
        self.sys_model=sys_model()
        self.constraints=constr()
        self.K1 =self.sys_model.sys_param.Kp+(self.parameters.Ts/2)  *self.sys_model.sys_param.Ki +(1/self.parameters.Ts)  * self.sys_model.sys_param.Kd
        self.K2 =-self.sys_model.sys_param.Kp+(self.parameters.Ts/2)  *self.sys_model.sys_param.Ki -(2/self.parameters.Ts)  * self.sys_model.sys_param.Kd
        self.K3= self.sys_model.sys_param.Kd/(self.parameters.Ts)
        self.args={}
        # self.args['lbx']=self.parameters.bounds.x_min*(self.parameters.N-1)+self.parameters.bounds.u_min*(self.parameters.N-1)+self.parameters.bounds.p_min*(self.parameters.N)
        # self.args['ubx']=self.parameters.bounds.x_max*(self.parameters.N-1)+self.parameters.bounds.u_max*(self.parameters.N-1)+self.parameters.bounds.p_max*(self.parameters.N)
        # self.args['lbg']=[0.0]*(((self.parameters.N-1)*(self.parameters.n_states+self.parameters.n_inputs)))
        # self.args['ubg']=self.args['lbx']
        self.args['lbx']=self.constraints.lbz
        self.args['ubx']=self.constraints.ubz
        self.args['lbg']=self.constraints.g_lb
        self.args['ubg']=self.args['lbg']
        self.opts = {'ipopt.linear_solver':'ma57','ipopt.warm_start_init_point':'yes' }
        # self.solver=self.set_mpc()
        # @property
        # def build_mpc(): 
        #     pass

        # @property.setter
        self.nlpsol1_=self.create_MPC()


    
    def create_MPC(self):
        # global self.nlpsol_
        # def set_MPC():
        #     pass
        xk = self.parameters.Sym_p.p_x0#SX.sym('x_init',2)
        u0  = self.parameters.Sym_p.p_u0
        
        e2=self.parameters.Sym_p.p_error[2]
        e1=self.parameters.Sym_p.p_error[1]
        #e0=self.parameters.Sym_p.p_error[0]
        x=[];
        u=[];
        g1=[];
        g2=[];
        e=[]
        J = 0;
        e0=(-self.sys_model.sys_output(xk)+self.parameters.Sym_p.p_ref[0])
        for k in range(self.parameters.N-1):
            uk=SX.sym('u_'+str(k),self.parameters.n_inputs)
            xkp1=SX.sym('x_' + str(k+1), self.parameters.n_states)

            
            g1      += [-xkp1+self.sys_model.sys_sim(xk,uk)]  #self.sys_model.sys_param.Ad@xk+self.sys_model.sys_param.Bd@uk]
            u       +=[uk]
            x       +=[xkp1]

            #y       +=[Cd@xkp1]
            
            g2      += [-uk+u0+self.K1*e0+self.K2*e1+self.K3*e2]
            # tmp = -uk+self.sys_model.PID_sim(self.parameters.Ts,e0,e1,e2,u0)
            e2 = e1
            e1 = e0
            ## error updates and output updates for the next iteration
            e0 = (-self.sys_model.sys_param.Cd@xkp1 + self.parameters.Sym_p.p_ref[k+1])  
            ey = (-self.sys_model.sys_param.Cd@xkp1 + self.parameters.Sym_p.p_ref_output[k+1])
            xk=xkp1
            u0=uk
            J=J+10*(ey*ey)# *(N+1-k)   ####### *(N+1-k) <--- use this for scaling purposes
            # pvec=veccat(self.parameters.Sym_p.p_u0,self.parameters.Sym_p.p_x0,self.parameters.Sym_p.p_error,self.parameters.Sym_p.p_ref_output)
        pvec=self.parameters.Sym_p.p_vec
        z = x+u + [self.parameters.Sym_p.p_ref]
        self.z =z
        self.g=g1+g2
        self.nlp_prob={'f':J,'x':veccat(*self.z),'g':veccat(*self.g),'p':pvec}
        return nlpsol('solver','ipopt',self.nlp_prob,self.opts)
        
    def set_mpc(self,z0,p0):

       self.a=self.nlpsol1_(x0=z0,p=p0,lbx=self.args['lbx'],ubx=self.args['ubx'],lbg=self.args['lbg'],ubg=self.args['ubg'])

       return self.a['x'].full().flatten()
        # return self.sol

