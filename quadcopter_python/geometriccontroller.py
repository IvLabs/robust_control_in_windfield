import numpy as np
import model.params as params
from math import sin, cos
from utils.utils import RPYToRot, RotToQuat, RotToRPY, vee_operator, hat_operator
from scipy.misc import derivative

kx = 16
kv = 5.6
kR = 8.81
kW = 2.54
Rd_i = np.eye(3)
def run_gmc(quad, des_state,t):
    global Rd_i
    #print(t)
    #if t == 0.0:
     #   Rd_i = np.eye(3)

    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    R = RPYToRot(phi,theta,psi) 
    p, q, r = quad.omega()
    omega = np.array([p,q,r]).T

    indx = int(t*160)

    des_x, des_y, des_z = des_state[indx,0:3]
    des_x_dot, des_y_dot, des_z_dot = des_state[indx,3:6]
    des_x_ddot, des_y_ddot, des_z_ddot = des_state[indx,6:9]
    des_psi = 0
    des_psi_dot = 0

    b1d = np.array([1,0,0]).T

    ex = des_state[indx,0:3].T - np.array([x,y,z]).T
    ev = des_state[indx,3:6].T - np.array([x_dot,y_dot,z_dot]).T
    des_accln = des_state[indx,6:9].T

    b3d = (-kx*ex - kv*ev - params.mass*(np.array([0,0,params.g]).T - des_accln))/     \
        (np.linalg.norm(-kx*ex - kv*ev - params.mass*(np.array([0,0,params.g]).T + des_accln)))

    b2d = np.cross(b3d,b1d)/np.linalg.norm(np.cross(b3d,b1d))
    Rd = np.c_[np.cross(b2d,b3d),b2d,b3d]
    Rd_dot = (Rd - Rd_i)*160
    Rd_i = Rd

    eR = vee_operator(Rd.T.dot(R) - R.T.dot(Rd))/2
    eW = np.array([p,q,r]).T - vee_operator(R.T.dot(Rd_dot))

    F = (-kx*ex - kv*ev - params.mass*(np.array([0,0,params.g]).T - des_accln)).dot(R.dot(np.array([0,0,1]).T))

    M = np.reshape(-kR*eR,(3,1)) - np.reshape(kW*eW,(3,1)) + np.reshape(np.cross(omega, params.I.dot(omega)),(3,1)) - params.I.dot(vee_operator(hat_operator(omega).dot(R.T.dot(Rd_dot))))
    
    
    return F,M






