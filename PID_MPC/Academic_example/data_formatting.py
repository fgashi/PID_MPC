import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import correlate
from scipy.stats import pearsonr
file1=open('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/results_z_r/results_N_100.txt','r')                                                                 #     y    r  e   x1   x2   u   t   ref
file2=open('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/results_pid_k/results_100.txt','r')           #     y    ref  e   x1   x2   u   t   
#file3=open('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/results_const_pid/results.txt','r')  #  header='  y     e   x1   x2   u   t   ref

matrix_academ=np.loadtxt(file1,skiprows=1)
matrix_pid_k=np.loadtxt(file2,skiprows=1)
#matrix_const_pid=np.loadtxt(file3,skiprows=1)
tgrid=matrix_academ[:,6]

y_academ=matrix_academ[:,0]
e_academ=matrix_academ[:,2]
u_academ=matrix_academ[:,5]
r_academ=matrix_academ[:,1]


y_pid_k=matrix_pid_k[:,0]
e_pid_k=matrix_pid_k[:,2]
u_pid_k=matrix_pid_k[:,5]

file1.close()
file2.close()
#file3.close()

plt.figure(1)
plt.step(tgrid,r_academ-matrix_academ[:,7],'r')#,plt.show(),
plt.grid()
plt.show()
# plt.figure(2)
# plt.plot(tgrid,matrix_academ[:,0],'r',tgrid,matrix_pid_k[:,0],tgrid,matrix_academ[:,7])#,plt.show(),
# plt.grid()
# plt.figure(3)
# plt.plot(tgrid,matrix_pid_k[:,1],'r',tgrid,matrix_pid_k[:,0],tgrid,matrix_academ[:,7])#,plt.show(),
# plt.grid()

# plt.figure(4)
# plt.plot(tgrid,matrix_academ[:,2],'k',tgrid,matrix_pid_k[:,2])#,plt.show(),
# plt.grid()

# p1,_=pearsonr(matrix_academ[:,0],matrix_const_pid[:,0])
# p2,_=pearsonr(matrix_academ[:,0],matrix_academ[:,0])
# # print('Pearsons correlation: %.3f' % p)
# p=np.array((p1,p2))
# print('\n', p)
# plt.figure(5)
# plt.plot(tgrid,matrix_academ[:,0]-matrix_pid_k[:,0],'k')

# plt.grid()
# plt.figure(5)
# plt.plot(tgrid,matrix_academ[:,2]-matrix_pid_k[:,2],'k')
# plt.grid()

# plt.show()
delta_y=y_academ-y_pid_k
delta_u=u_academ-u_pid_k
delta_e=e_academ-e_pid_k
delta_r=r_academ-y_academ


delta_matrix=(np.matrix((delta_y,delta_e,delta_u,delta_r,tgrid)).T)



np.savetxt('/home/fatos/python_ws/git_files/overleaf_files/Graphics/Academic_example/Comparison/results.txt', delta_matrix, delimiter='  ',header='  delta_y       delta_e       delta_u       delta_ref       t  ' ,comments='')


print('here')