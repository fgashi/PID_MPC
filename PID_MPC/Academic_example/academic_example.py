from casadi import *
from casadi.tools import *  # for dotdraw
import numpy as np
import matplotlib.pyplot as plt

# import sys
# sys.path.append('/home/fatos/cpp_ws/publications/isr_robotics/reference_generation/Parameters')
# # from ..Parameters.parameters import *
# from Parameters.parameters import parameters
#from Model.model import sys_model
# from Configuration import configuration
from MPC.mpc import MPC
from Reference_generator.MPC_init import mpc_init

n_case=5

tst=MPC()

N=tst.parameters.N
Ts=tst.parameters.Ts
n_states=tst.parameters.n_states
n_inputs=tst.parameters.n_inputs
u_0=0
x_init=np.matrix(([[0],[0]]))
# Reference generator function ref_generator(amplitude,frequency,Constant(true or false))
T=True
ref_gen=tst.parameters.ref_params.ref_generator(3,1./200,T)
for j in range(len(ref_gen)):
    if j >= (int(len(ref_gen)/2)):
        ref_gen[j]=-ref_gen[j]
    else:
        pass
pv1=[0.0]*(5)
# pv1[1]=-5
pv1[2]=-tst.sys_model.sys_output(x_init)+ref_gen[0]
pv1[1:2]=x_init
pv=vertcat(pv1,ref_gen)

# zv=np.linspace(0,0,((tst.parameters.n_states+tst.parameters.n_inputs)*(N-1))+N)
zv=np.zeros(((3*(N-1)+N),1))

an=tst.set_mpc(zv,pv)
length=len(an)
x1_opt = an[0:(n_states*(N-1)):n_states]
x2_opt = an[1:(n_states*(N-1)):2]
u1_opt = an[n_states*(N-1):(n_states*(N-1)+(n_inputs*(N-1))):n_inputs]
x1=vertcat(pv1[1],x1_opt)
x2=vertcat(pv1[2],x2_opt)
u1=vertcat(u1_opt,DM.nan(1))
output=np.matrix((np.zeros((N,1))))
error=np.matrix((np.zeros((N,1))))
x0=(np.hstack((x1,x2)).T)
for j in range(N):
    output[j,:]=tst.sys_model.sys_output(x0[:,j].T)
    error[j,:]=ref_gen[j]-output[j,:]

tgrid = np.arange(0,N*Ts,Ts)

ref_opt=an[length-N-1:length-1]

t_wall_solver=np.matrix((tst.nlpsol1_.stats()['t_wall_total']))
x_results=np.matrix(( np.hstack((x1,x2 )) ))
u_results=np.matrix((u1))
total_results=np.hstack((output,np.matrix((ref_opt)).T,error,x_results,u_results,np.matrix((tgrid)).T ,np.matrix((ref_gen)).T ))
print(" saving the data: \n")

K_final=[tst.sys_model.sys_param.Kp,tst.sys_model.sys_param.Ki, tst.sys_model.sys_param.Kd]
np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/results_z_r/cases/results_N_'+str(N)+'_case_'+str(n_case)+'.txt',total_results, delimiter='  ',header='  y    r  e   x1   x2   u   t   ref' ,comments='')
np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/results_z_r/cases/sol_time_N_'+str(N)+'_case_'+str(n_case)+'.txt',t_wall_solver, delimiter='  ',header='  t_wall_solver ' ,comments='') 
np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/results_z_r/cases/K_params_N_'+str(N)+'_case_'+str(n_case)+'.txt',K_final, delimiter='  ',header='  Kp         Ki      Kd ' ,comments='') 
