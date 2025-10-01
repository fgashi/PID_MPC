from casadi import *
import numpy as np
from Robot_model.R_model import Robotino_
from Motor.robotino_motor import robotino_motor
from Constraints_robotino.constraints_r import constraints_r
from Constraints_robotino.constraints_m import constraints_m
from MPC.mpc_rm import mpc_rm
from matplotlib import pyplot as plt
from ref_gen.ref_gen import ref_gen

rob=mpc_rm()
N=rob.sym_m.N
Ts=rob.sym_m.Ts
x_d=np.zeros((3,rob.sym_m.N))
# x_d[0,:]=0
# x_d[2,:]=0
n = [Ts/N*k for k in range(N)]

amplitude=1.0
frequency=1/400

T=True
for i in range(len(n)):
    #temp = (np.cos(2*pi*i/10) if n[i]>=3*Ts/N else 0)
    if T==True:
        x_d [0,i] = (amplitude*np.cos(2*pi*i*frequency) if n[i]>=0 else 0)#) )
    else:
        x_d[0,i] = (amplitude)
        pass

# x_d[0,:]=2
x_init=np.zeros((3,1))
x_mot_init=np.zeros((15,1))
x_init_r=veccat(x_init,x_d)

# x_graf,y_graf=ref_gen(0,N*Ts,Ts)



pv=veccat(x_init_r,x_mot_init)
zv=np.zeros(((rob.n_states_m+rob.n_states_r+rob.sym_m.n_inputs+9)*(rob.sym_m.N-1)+((rob.sym_m.n_motors)*rob.sym_m.N),1))
am=rob.solve_mpc_rm(zv,pv)
t1_wall=np.matrix((rob.mpc.stats()['t_wall_total']))
bm=rob.solve_mpc_rm(am,pv)
print('Warm start: \n \n ')

tgrid = np.arange(0,(rob.sym_m.N)*rob.sym_m.Ts,rob.sym_m.Ts)
position=bm[0:(N-1)*3]
x0=position[0::3]
x1=position[1::3]
x2=position[2::3]
x_0=np.append(x_init[0,0],x0)
x_1=np.append(x_init[1,0],x1)
x_2=np.append(x_init[2,0],x2)

position_res=np.hstack((np.matrix((x_0)).T,np.matrix((x_1)).T,np.matrix((x_2)).T ))


states=bm[(N-1)*3:(N-1)*9]
i_0=np.append(x_mot_init[0,0],states[0::6])
i_1=np.append(x_mot_init[1,0],states[1::6])
i_2=np.append(x_mot_init[2,0],states[2::6])

omega_0=np.append(x_mot_init[3,0],states[3::6])
omega_1=np.append(x_mot_init[4,0],states[4::6])
omega_2=np.append(x_mot_init[5,0],states[5::6])

states_res=np.hstack((np.matrix((i_0)).T,np.matrix((i_1)).T,np.matrix((i_2)).T,np.matrix((omega_0)).T,np.matrix((omega_1)).T,np.matrix((omega_2)).T ))

inputs=bm[(N-1)*9:(N-1)*9+3*(N-1)]

u_0=np.append(x_mot_init[6,0],inputs[0::3])
u_1=np.append(x_mot_init[7,0],inputs[1::3])
u_2=np.append(x_mot_init[8,0],inputs[2::3])

inputs_res=np.hstack((np.matrix((u_0)).T,np.matrix((u_1)).T,np.matrix((u_2)).T))

references=bm[(N-1)*9+3*(N-1):(N-1)*9+3*(N-1)+3*N]

ref_0=references[0::3]
ref_1=references[1::3]
ref_2=references[2::3]

ref_res=np.hstack((np.matrix((ref_0)).T,np.matrix((ref_1)).T,np.matrix((ref_2)).T,np.matrix((tgrid)).T))


K1_total=bm[(N-1)*9+3*(N-1)+3*N:(N-1)*9+3*(N-1)+3*N+3*(N-1)]
K11=K1_total[0::3]
K12=K1_total[1::3]
K13=K1_total[2::3]

K_res=np.hstack((np.matrix((K11)).T,np.matrix((K12)).T,np.matrix((K13)).T,np.matrix((tgrid[0:-1])).T))


total_results=np.hstack((position_res,states_res,inputs_res,ref_res))

reference=np.hstack((x_d.T,np.matrix((tgrid)).T))

t2_wall_sol=np.matrix((rob.mpc.stats()['t_wall_total']))

if T==False:
    reference_gen='is_constant'
    ref_amplitude=amplitude
    ref_frequency=0*frequency
else:
    reference_gen='is not constant'
    ref_amplitude=amplitude
    ref_frequency=frequency


sol_setup=np.hstack(( np.matrix((t2_wall_sol)),t1_wall,np.matrix((ref_amplitude)),np.matrix((ref_frequency)),np.matrix((rob.sym_m.Q[0,0])),np.matrix((rob.sym_m.Q[1,1])),np.matrix((rob.sym_m.Q[2,2])), ))


np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Robotino_example/pid_k/results_N_'+str(N)+'.txt',total_results, delimiter='  ',header='  x1    x2    x3    i1    i2    i3    omega_1    omega_2    omega_3    input_1    input_2    input_3    ref_1    ref_2    ref_3        t' ,comments=' ')
np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Robotino_example/pid_k/K_res_N_'+str(N)+'.txt',K_res, delimiter='  ',header='      K1    K2    K3     t   ' ,comments=' ')
np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Robotino_example/pid_k/ref_N_'+str(N)+'.txt',reference, delimiter='  ',header='  x1_d    x2_d    x3_d      t' ,comments=' ')
np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Robotino_example/pid_k/sol_setup_N_'+str(N)+'.txt',sol_setup, delimiter='  ',header='  t_wall_warm_start      t_wall_non_warm    ref_amplitude    ref_frequency    Q11    Q12   Q13'    ,comments=' ')
# np.savetxt('/home/fatos/python_ws/ref_MPC/Roboino_example_K/robotino_files/Results/ref_setup_N_+reference_gen+.txt',reference_gen, delimiter='  ',header='  Is_constant  ',comments='')


# np.savetxt('/home/fatos/python_ws/ref_MPC/Academic example/Results/sol_time_N_'+str(N)+'.txt',t_wall_solver, delimiter='  ',header='  t_wall_solver ' ,comments='') 
