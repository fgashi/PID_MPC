# from Configuration_robotino.config_r import conf_r
from Parameters_robotino.parameters_robotino import parameters_rm
from Configuration_robotino.config_m import conf_m


class constraints_m(parameters_rm,conf_m):
    def __init__(self):
        super().__init__()
        

        self.g_lb_m = [0.0]*( (self.N-1)*(self.n_states+self.n_inputs))
        self.g_ub_m = self.g_lb_m;

        # self.h_lb = self.PID_min*(self.N-1)
        # self.h_ub = self.PID_max*(self.N-1)
        self.lbz_m  = self.bounds_m.eta_min*(self.N-1) + self.bounds_m.u_min * ((self.N-1)) + self.bounds_m.r_min*(self.N)  + self.bounds_m.K_min*(self.N-1) 
        self.ubz_m  = self.bounds_m.eta_max*(self.N-1) + self.bounds_m.u_max * ((self.N-1)) + self.bounds_m.r_max*(self.N)   +self.bounds_m.K_max*(self.N-1)