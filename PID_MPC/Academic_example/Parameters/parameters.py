from casadi import*
# import sys
# sys.path.append('/home/fatos/cpp_ws/publications/isr_robotics/reference_generation/structured_proj')
# from structured_proj.configuration import configuration
# from ..configuration import configuration
from Configuration.configuration import conf
class parameters(conf): 
    
    def __init__(self):
        super().__init__()
        self.bounds=boundaries()
        self.ref_params=ref_param()
        self.Sym_p=Sym_p()
    
    

    



class boundaries:
    def __init__(self):
        self.x_min =[-300,-600];
        self.x_max =[300,600]

        self.p_min =[-600]
        self.p_max = [600]

        # K_min = [-255,-255,-255]
        # K_max = [+255,+255,+255]

        self.u_min = [-55]#,-50];
        self.u_max = [55]#,50]

        self.PID_min = [0,0,0];
        self.PID_max = [500,500,500];

class ref_param(conf):
    def __init__(self):
        super().__init__()
        self.ref_p=[]
        self.n_count=[self.Ts/self.N*k for k in range(self.N)]
    
    def ref_generator(self,amplitude,frequency,constant_ref):

        n = [self.Ts/self.N*k for k in range(self.N)]
        
        if constant_ref==True:
            for i in range(len(n)):
                temp=(amplitude)
                self.ref_p.append(temp)
            return self.ref_p
            
        else:
            for i in range(len(n)):    
                temp=(amplitude*np.cos(2*np.pi*i*frequency)if n[i]>=0 else 0)
                self.ref_p.append(temp)
            return self.ref_p


class Sym_p(conf):
    def __init__(self):
        super().__init__()
        self.p_x0=SX.sym('p_x0',self.n_states)

        self.p_u0=SX.sym('p_u0',self.n_inputs)

        # Decision variable r*
        self.p_ref=SX.sym('p_ref',(self.N))
        
        # Parameters for the output to be followed
        self.p_ref_output=SX.sym('p_output',(self.N))

        self.p_error=SX.sym('p_error',3)
        self.xk = self.p_x0#SX.sym('x_init',2)
        self.p_vec=vertcat(self.p_u0,self.p_x0,self.p_error,self.p_ref_output)



