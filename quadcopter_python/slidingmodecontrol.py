import numpy as np
import model.params as params
from math import *

Ix = params.I[0,0]
Iy = params.I[1,1]
Iz = params.I[2,2]
m  = params.mass
g  = params.g
l  = params.arm_length
minF = params.minF
maxF = params.maxF


def run_smc(quad, des_state,t):

    # Import the desired and Current state of the Quadcopter
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    phi_dot, theta_dot, psi_dot = quad.omega()

    indx = int(t*160)

    des_x, des_y, des_z = des_state[indx,0:3]
    des_x_dot, des_y_dot, des_z_dot = des_state[indx,3:6]
    des_x_ddot, des_y_ddot, des_z_ddot = des_state[indx,6:9]
    des_psi = 0
    des_psi_dot = 0
    des_phi = 0 
    des_phi_dot = 0
    des_theta = 0 
    des_theta_dot = 0
    des_phi_ddot = 0 
    des_psi_ddot = 0 
    des_theta_ddot = 0


    # Define the error and the error derivative term
    e1 = des_z - z 
    e1_dot = des_z_dot - z_dot

    e2 = des_phi - phi 
    e2_dot = des_phi_dot - phi_dot

    e3 = des_theta - theta 
    e3_dot = des_theta_dot - theta_dot

    e4 = des_psi - psi 
    e4_dot = des_psi_dot - psi_dot

    # Tuning Parameters 
    lamb = [50, 50, 50, 50]
    Kd = [20, 20, 20, 20]
    delta = 10

    # Selection of Sliding surface
    s1 = e1_dot + lamb[0] * e1
    s2 = e2_dot + lamb[1] * e2
    s3 = e3_dot + lamb[2] * e3
    s4 = e4_dot + lamb[3] * e4

    # Switching Parameter
    uD1 = s1/(abs(s1) + delta)
    uD2 = s2/(abs(s2) + delta)
    uD3 = s3/(abs(s3) + delta)
    uD4 = s4/(abs(s4) + delta)
    
    # Thrust
    U1 = (g + lamb[0] * e1_dot + des_z_ddot) * (m/(cos(phi) * cos(psi))) + Kd[0] * uD1

    if U1 > maxF:
        U1 = maxF
    elif U1 < minF:
        U1 = minF

    # Matrix to store moments for pitch, roll and yaw
    M = np.array([(lamb[1] * (des_phi_dot - phi_dot) + des_phi_ddot - theta_dot * psi_dot * ((Iy - Iz)/Ix)) * (Ix / l) + uD2, 
                  (lamb[2] * (des_theta_dot - theta_dot)+ des_theta_ddot - phi_dot * psi_dot * ((Iz - Ix)/Iy)) * (Iy / l) + uD3,
                  (lamb[3] * (des_psi_dot - psi_dot) + des_psi_ddot - phi_dot * theta_dot * ((Ix - Iy)/Iz)) * Iz + uD4])
    
    return U1, M