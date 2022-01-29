
import numpy as np
import model.params as params
import cvxpy as cp
import control



class MpcController:
    def __init__(self,quad):
        
        self.x0 = np.zeros(12)
        
        self.N = 20
        self.A = quad.A()
        self.B = quad.B()
        self.C = quad.C()
        self.D = quad.D()
        [self.nx, self.nu] = self.B.shape
        self.x_current = cp.Variable((self.nx, self.N+1))
        self.u = cp.Variable((self.nu, self.N))
        self.x_init = cp.Parameter(self.nx)
        self.A_zoh = np.eye(12)
        self.B_zoh = np.zeros((12, 4))
        self.zoh()
        self.x_reference = np.zeros((12,self.N))
        self.constr = []
        self.cost = 0. 
        

        self.Q = np.array([[1000,0,0,0,0,0,0,0,0,0,0,0]
                          ,[0,1000,0,0,0,0,0,0,0,0,0,0]
                          ,[0,0,1000,0,0,0,0,0,0,0,0,0]
                          ,[0,0,0,1,0,0,0,0,0,0,0,0]
                          ,[0,0,0,0,1,0,0,0,0,0,0,0]
                          ,[0,0,0,0,0,1,0,0,0,0,0,0]
                          ,[0,0,0,0,0,0,100,0,0,0,0,0]
                          ,[0,0,0,0,0,0,0,100,0,0,0,0]
                          ,[0,0,0,0,0,0,0,0,100,0,0,0]
                          ,[0,0,0,0,0,0,0,0,0,1,0,0]
                          ,[0,0,0,0,0,0,0,0,0,0,1,0]
                          ,[0,0,0,0,0,0,0,0,0,0,0,1]])

        self.R = np.eye(4)*1
        self.formconstraint()
        
    def zoh(self):
        ''' Converts continuous time state-space model into discrete time state space model'''
        
        sys = control.StateSpace(self.A, self.B, self.C, self.D)
        sys_discrete = control.c2d(sys, 0.00625, method='zoh')

        # Discrete time State-transition matrices
        self.A_zoh = np.array(sys_discrete.A)
        self.B_zoh = np.array(sys_discrete.B)

    def formconstraint(self):
        for t in range(self.N):
           self.constr += [self.x_current[:, t + 1] == self.A_zoh@self.x_current[:, t] + self.B_zoh@(self.u[:, t]-np.array([params.mass*params.g,0,0,0]))]
           self.cost = cp.quad_form(self.u[:, t]-np.array([params.mass*params.g,0,0,0]), self.R)

    def run_mpc(self, des_state,t):
        '''Formulates mpc based control optimization problem.
        rx : reference state, cx : current state, constr : constraints of optimization problem
        cost : Quadratic objective cost'''
        indx = int(t*160)
        
        rc = des_state[indx:indx+self.N,0:3].shape[0]
    
        self.x_reference[0:3,0:rc] = des_state[indx:indx+self.N,0:3].T

        self.x_reference[6:9,0:rc] = des_state[indx:indx+self.N,3:6].T

       

        self.x_reference[3:6,:] = np.zeros((3,self.N))
        self.x_reference[9:12,:] = np.zeros((3,self.N))

        if rc < self.N:
            self.x_reference[:,rc:self.N] = np.tile(self.x_reference[:,rc-1],(self.N-rc,1)).T

        cost = self.cost
        
        constr = [self.x_current[:, 0] == self.x_init]
        constr += self.constr
        for t in range(self.N):

            # Linear Quadratic cost
            cost += cp.quad_form(self.x_reference[:,t] - self.x_current[:, t], self.Q)  # Linear Quadratic cost
            
            # constraint: x(t+1) = Ax(t) + Bu(t)
            #constr += [self.x_current[:, t + 1] == self.A_zoh@self.x_current[:, t] + self.B_zoh@(self.u[:, t]-np.array([params.mass*params.g,0,0,0]))]

        cost += cp.quad_form(self.x_current[:, self.N] - self.x_reference[:,self.N-1], self.Q)  # End of trajectory error cost
       
        problem = cp.Problem(cp.Minimize(cost), constr)

        
        return problem
