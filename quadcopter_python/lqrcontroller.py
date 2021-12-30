import numpy as np
import model.params as params
from math import sin, cos
import cvxpy as cp
import control
from model.quadcopter import Quadcopter
from scipy.integrate.quadpack import quad
import scipy

class LqrController:
    def __init__(self, quad):
        self.rx = np.zeros(12)
        self.x0 = np.zeros(12)
        
        self.N = 20
        self.A = quad.A()
        self.B = quad.B()
        self.C = quad.C()
        self.D = quad.D()
        self.K = np.zeros((4,1))
        self.kmatrix()

    def Q(self):
        # State cost
        Q = np.eye(12)*0.01
        Q[0,0] =1000
        Q[1,1] = 1000
        Q[2, 2] = 1000 
        Q[6, 6] = 100.  
        Q[7, 7] = 100.  
        Q[8, 8] = 100.  
        return Q

    def R(self):
        # Actuator cost
        R = np.eye(4)*1
        return R

    def kmatrix(self):

        S = scipy.linalg.solve_continuous_are(self.A, self.B, self.Q(), self.R())
        K = scipy.linalg.inv(self.R()).dot(self.B.T.dot(S))
        self.K = K




    def run_lqr(self, des_state,quad,t):
        indx = int(t*160)
        des_x, des_y, des_z = des_state[indx,0:3]
        des_x_dot, des_y_dot, des_z_dot = des_state[indx,3:6]
        des_psi = 0
        des_psi_dot = 0
        rx = np.array([des_x,des_y,des_z,0,0,des_psi,des_x_dot,des_y_dot,des_z_dot,0,0,des_psi_dot])

        
        rpy = quad.attitude()
        cx = np.array([quad.state[0],quad.state[1],quad.state[2],rpy[0],rpy[1],rpy[2],quad.state[7],quad.state[8],quad.state[9],quad.state[10],quad.state[11],quad.state[12]])
        
        U = -self.K.dot(cx-rx)

        return U


