# from Configuration_robotino.config_r import conf_r
from Parameters_robotino.parameters_robotino import parameters_rm
from Configuration_robotino.config_r import conf_r


class constraints_r(parameters_rm,conf_r):
    def __init__(self):
        super().__init__()
        

        self.g_lb_r = [0.0]*(((self.N-1)*(self.n_states)))#+self.n_inputs)));
        self.g_ub_r = self.g_lb_r;

        # self.h_lb = self.PID_min*(self.N-1)
        # self.h_ub = self.PID_max*(self.N-1)
        self.lbz_r  = self.bounds_r.x_min*(self.N-1)# + self.bounds_r.w_min * ((self.N-1)) + self.bounds_r.pr_min*(self.N)   #+PID_min
        self.ubz_r  = self.bounds_r.x_max*(self.N-1)# + self.bounds_r.w_max * ((self.N-1)) + self.bounds_r.pr_max*(self.N)   #+PID_max
        

        # self.lbz_m = self.bounds_m.i_min*(self.N-1) + self.bounds_m.u_min * ((self.N-1)) + self.bounds_m.p_min*(self.N)   
        # self.ubz_m = self.bounds_m.i_min*(self.N-1) + self.bounds_m.u_min * ((self.N-1)) + self.bounds_m.p_max*(self.N) 

