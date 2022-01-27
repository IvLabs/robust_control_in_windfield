
import numpy as np
import model.params as params
from math import sin, cos, sqrt

# SMC Controller for quacopter
# Return [F, M] F is total force thrust, M is 3x1 moment matrix

# Constants
k_d_x = 30
k_p_x = 3
k_d_y = 30
k_p_y = 3
k_p_z = 1000
k_d_z = 200
k_p_phi = 160
k_d_phi = 3
k_p_theta = 160
k_d_theta = 3
k_p_psi = 80
k_d_psi = 5
lamda1 = 68
lamda2 = 5
lamda3 = 5
lamda4 = 5
kd1 = 25
kd2 = 100
kd3 = 100
kd4 = 100 
delta = 0.3
Ix = 0.00025
Iy = 0.000232
Iz = 0.0003738
Jr = 0
l = params.L


def smc(quad, des_state,t):


    maxF=params.maxF
    minF=params.minF

    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()

    p, q, r = quad.omega()
    phi_dot = p
    theta_dot = q 
    psi_dot = r

    

    indx = int(t*160)
    
    
    des_x, des_y, des_z = des_state[indx,0:3]
    des_x_dot, des_y_dot, des_z_dot = des_state[indx,3:6]
    des_x_ddot, des_y_ddot, des_z_ddot = des_state[indx,6:9]
    des_psi = 0
    des_psi_dot = 0
    #des_phi = 0
    des_phi_dot = 0
    #des_theta = 0
    des_theta_dot = 0

        # Commanded accelerations
    commanded_r_ddot_x = des_x_ddot + k_d_x * (des_x_dot - x_dot) + k_p_x * (des_x - x)
    commanded_r_ddot_y = des_y_ddot + k_d_y * (des_y_dot - y_dot) + k_p_y * (des_y - y)
    commanded_r_ddot_z = des_z_ddot + k_d_z * (des_z_dot - z_dot) + k_p_z * (des_z - z)

    des_phi = 1 / params.g * (commanded_r_ddot_x * sin(des_psi) - commanded_r_ddot_y * cos(des_psi))
    des_theta = 1 / params.g * (commanded_r_ddot_x * cos(des_psi) + commanded_r_ddot_y * sin(des_psi))

    
    s1 = des_z_dot - z_dot + lamda1 * (des_z - z )
    s2 = des_phi_dot - phi_dot + lamda2 * (des_phi - phi)
    s3 = des_theta_dot - theta_dot + lamda3 * (des_theta - theta)
    s4 = des_psi_dot - psi_dot + lamda4 * (des_psi - psi)

    #print(s)
    # Thrust
    
    F = (params.mass * (params.g + des_z_ddot + lamda1 * (des_z_dot - z_dot)))/(cos(phi)*cos(psi)) + kd1 * (s1/(np.absolute(s1) + delta))
   
    

    M = np.array([[(lamda2 * (phi_dot - des_phi_dot) - (theta_dot * psi_dot * ((Iy - Iz)/Ix)))*(Ix/l) + kd2 * (s2/(np.absolute(s2) + delta)) ,
                  (lamda3 * (theta_dot - des_theta_dot) - (phi_dot * psi_dot * ((Iz - Ix)/Iy)))*(Iy/l)  + kd3 * (s3/(np.absolute(s3) + delta)) ,
                  (lamda4 * (psi_dot - des_psi_dot) - (theta_dot * phi_dot * ((Ix - Iy)/Iz)) + kd4 * (s4/(np.absolute(s4) + delta)))*Iz ]]).T
    #M = np.array([[(lamda2 * (des_phi_dot - phi_dot ) + kd2 * (s2/(np.absolute(s2) + delta))) ,
                   #(lamda3 * (des_theta_dot - theta_dot) + kd3 * (s3/(np.absolute(s3) + delta))) ,
                   #(lamda4 * (des_psi_dot - psi_dot)  + kd4 * (s4/(np.absolute(s4) + delta)))*Iz ]]).T
    

    des_rpy = np.array([des_phi, des_theta, des_psi])

    return F, M, des_rpy
