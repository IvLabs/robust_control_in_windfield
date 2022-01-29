
import numpy as np
import model.params as params
from utils.utils import  vee_operator, hat_operator
from utils.quaternion import Quaternion


kx = 1.7
kv = 12.8
kR = 0.8
kW = 0.04
dt = 1/160
acclns = np.zeros((1,3))
def run_gmc(quad, des_state,t):
    global acclns
    indx = int(t*160)
    acclns = np.r_[acclns,quad.accel.reshape((1,3))]
   
    desacclns = des_state[:,6:9]
    desjerks  = np.diff(desacclns,axis=0)/dt
    dessnaps  = np.diff(desjerks,axis=0)/dt
    desjerks =  np.r_[desjerks,np.zeros((2,3))]
    dessnaps = np.r_[dessnaps,np.zeros((2,3))]

    x, y, z = quad.position()
    quat = np.array([quad.state[3],quad.state[4],quad.state[5],quad.state[6]])
    bRw = Quaternion(quat).as_rotation_matrix()
    R = bRw.T   # body to world R


    x_dot, y_dot, z_dot = quad.velocity()
    p, q, r = quad.omega()
    omega = np.array([p,q,r]).T
    jerks = np.diff(acclns,axis=0)/dt
    jerk = jerks[indx,:]


    des_accln = desacclns[indx,:].T
    des_jerk = desjerks[indx,:].T
    des_snap = dessnaps[indx,:].T


    b1d = np.array([1,0,0]).T
    
    ex = des_state[indx,0:3].T - np.array([x,y,z]).T
    ev = des_state[indx,3:6].T - np.array([x_dot,y_dot,z_dot]).T
    ea = des_accln - quad.accel.reshape(3)
    
    ej = des_jerk - jerk
    
    A = -kx*ex - kv*ev + params.mass*np.array([0,0,-params.g]).T + des_accln

    b3c = -(A/np.linalg.norm(A))
    C = np.cross(b3c,b1d)
    b2c = -C/np.linalg.norm(C)
    b1c = np.cross(b2c,b3c)

    A_dot = -kx*ev - kv*ea + params.mass*des_jerk
    M = -A_dot/np.linalg.norm(A)
    
    b3c_dot = -A_dot/np.linalg.norm(A) + (A.dot(A_dot)/np.linalg.norm(A)**3)*A
    C_dot = np.cross(b3c_dot,b1d)
    b2c_dot = -C_dot/np.linalg.norm(C) + (C.dot(C_dot)/np.linalg.norm(C)**3)*C
    b1c_dot = np.cross(b2c_dot,b3c) + np.cross(b2c,b3c_dot)

    A_ddot = -kx*ea - kv*ej + params.mass*des_snap
    b3c_ddot = -A_ddot/np.linalg.norm(A) + (2*A.dot(A_dot)/np.linalg.norm(A)**3)*A_dot + \
        ((np.linalg.norm(A_dot)**2 + A.dot(A_ddot))/np.linalg.norm(A)**3)*A - (3*(A.dot(A_dot))**2/np.linalg.norm(A)**5)*A
    C_ddot = np.cross(b3c_ddot,b1d)
    b2c_ddot = -C_ddot/np.linalg.norm(C) + (2*C.dot(C_dot)/np.linalg.norm(C)**3)*C_dot + \
        ((np.linalg.norm(C_dot)**2 + C.dot(C_ddot))/np.linalg.norm(C)**3)*C - (3*(C.dot(C_dot))**2/np.linalg.norm(C)**5)*C
    b1c_ddot = np.cross(b2c_ddot,b3c) + np.cross(2*b2c_dot,b3c_dot) + np.cross(b2c,b3c_ddot)


    Rc = np.c_[b1c,b2c,b3c]
    Rc_dot = np.c_[b1c_dot,b2c_dot,b3c_dot]
    Rc_ddot = np.c_[b1c_ddot,b2c_ddot,b3c_ddot]
   
    omegac = vee_operator(np.matmul(Rc.T,Rc_dot))
    omegac_dot = vee_operator(np.matmul(Rc.T,Rc_ddot) - np.linalg.matrix_power(hat_operator(omegac),2))

    eR = vee_operator(np.matmul(Rc.T,R) - np.matmul(R.T,Rc))/2
    eW = np.array([p,q,r]).T - np.matmul(R.T,np.matmul(Rc,omegac))

    F = -A.dot(R.dot(np.array([0,0,1]).T))


    U = Rc.T.dot(Rc_dot.dot(R.T.dot(Rc.dot(omegac))))
    U = np.reshape(U,(3,1))
    U = np.reshape((U - R.T.dot(Rc.dot(omegac_dot))),3)
   
    X = params.I@U.T
    Y = np.cross(omega, params.I.dot(omega))
   

    M = -kR*eR - kW*eW + Y - X 
    
    
    
    return F,M






