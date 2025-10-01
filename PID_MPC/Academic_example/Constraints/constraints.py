# from ..Configuration.configuration import conf
from Parameters.parameters import parameters
class constr: 

    def __init__(self):
        self.parameters=parameters()
        self.g_lb = [0.0]*(((self.parameters.N-1)*(self.parameters.n_states+self.parameters.n_inputs)));
        self.g_ub = self.g_lb;

        # self.h_lb = self.PID_min*(self.N-1)
        # self.h_ub = self.PID_max*(self.N-1)
        self.lbz  = self.parameters.bounds.x_min*(self.parameters.N-1) + self.parameters.bounds.u_min * ((self.parameters.N-1)) + self.parameters.bounds.p_min*(self.parameters.N)   #+PID_min
        self.ubz  = self.parameters.bounds.x_max*(self.parameters.N-1) + self.parameters.bounds.u_max * ((self.parameters.N-1)) + self.parameters.bounds.p_max*(self.parameters.N)   #+PID_max
        