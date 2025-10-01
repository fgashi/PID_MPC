
from casadi import *
from Configuration_robotino.config_r import conf_r
from Configuration_robotino.config_m import conf_m
class parameters_rm:
    def __init__(self) :
        super( ).__init__()
        self.bounds_r=boundaries_r()
        self.bounds_m=boundaries_m()
        # self.param_r=Sym_p_r()
        # self.param_m=Sym_p_m()
        
        pass


class boundaries_r:
    def __init__(self):
        self.x_min =[ -inf,  -inf,   -2*np.pi]
        self.x_max =[  inf,   inf,  2*np.pi]

        # self.pr_min =[-600]
        # self.pr_max = [600]



        # self.w_min = [ -3200,  -3200, -3200]
        # self.w_max = [  3200,   3200,  3200]


class boundaries_m:
    def __init__(self):
        self.eta_min =[-15, -15, -15, -4000,  -4000, -4000];
        self.eta_max =[ 15,  15,  15,  4000,   4000,  4000]


        # self.w_min = [ -3200,  -3200, -3200]
        # self.w_max = [  3200,   3200,  3200]

        self.r_min =[ -4000, -4000, -4000,]
        self.r_max =[  4000,  4000,  4000]



        self.u_min = [-24, -24, -24]#,-50];
        self.u_max = [ 24,  24,  24]




# class Sym_p_r(conf_r):
#     def __init__(self):
#         super().__init__()
#         self.p_x0=SX.sym('p_x0',self.n_states)

#         self.p_u0=SX.sym('p_u0',self.n_inputs)

#         # Decision variable r*
#         # self.p_ref=SX.sym('p_ref',(self.n_states,self.N))
        
#         # Parameters for the output to be followed
#         self.p_ref_output=SX.sym('p_output',(self.n_states,self.N))

#         self.p_error=SX.sym('p_error',3)
#         self.xk = self.p_x0#SX.sym('x_init',2)
#         self.p_vec=vertcat(self.p_u0,self.p_x0,self.p_error,self.p_ref_output)

# class Sym_p_m(conf_m):
#     def __init__(self):
#         super().__init__()
#         self.p_x0=SX.sym('p_x0',self.n_states)

#         self.p_u0=SX.sym('p_u0',self.n_inputs)

#         # Decision variable r*
#         self.p_m_ref=SX.sym('p_ref',(self.n_motors,self.N))
        
#         # Parameters for the output to be followed
#         self.p_ref_m_output=SX.sym('p_output',(self.n_motors,self.N))

#         self.p_error=SX.sym('p_error',3)
#         self.xk = self.p_x0#SX.sym('x_init',2)
#         # self.p_vec=vertcat(self.p_u0,self.p_x0,self.p_error,self.p_ref_output)
