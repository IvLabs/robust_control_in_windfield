

import numpy as np
import model.params as params
from math import sin, cos

# PD Controller for quacopter
# Return [F, M] F is total force thrust, M is 3x1 moment matrix

# Constants
k_p_x = 30
k_d_x = 3
k_p_y = 30
k_d_y = 3
k_p_z = 1000
k_d_z = 200
k_p_phi = 160
k_d_phi = 3
k_p_theta = 160
k_d_theta = 3
k_p_psi = 80
k_d_psi = 5

def run(quad, des_state,t):
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    p, q, r = quad.omega()
     
    indx = int(t*160)
    
    des_x, des_y, des_z = des_state[indx,0:3]
    des_x_dot, des_y_dot, des_z_dot = des_state[indx,3:6]
    des_x_ddot, des_y_ddot, des_z_ddot = des_state[indx,6:9]
    des_psi = 0
    des_psi_dot = 0
    # Commanded accelerations
    commanded_r_ddot_x = des_x_ddot + k_p_x * (des_x_dot - x_dot) + k_d_x * (des_x - x)
    commanded_r_ddot_y = des_y_ddot + k_p_y * (des_y_dot - y_dot) + k_d_y * (des_y - y)
    commanded_r_ddot_z = des_z_ddot + k_p_z * (des_z_dot - z_dot) + k_d_z * (des_z - z)

    # Thrust
    F = params.mass * (params.g + commanded_r_ddot_z)
    # Moment
    p_des = 0
    q_des = 0
    r_des = des_psi_dot
    des_phi = 1 / params.g * (commanded_r_ddot_x * sin(des_psi) - commanded_r_ddot_y * cos(des_psi))
    des_theta = 1 / params.g * (commanded_r_ddot_x * cos(des_psi) + commanded_r_ddot_y * sin(des_psi))

    M = np.array([[k_p_phi * (des_phi - phi) + k_d_phi * (p_des - p),
                   k_p_theta * (des_theta - theta) + k_d_theta * (q_des - q),
                   k_p_psi * (des_psi - psi) + k_d_psi * (r_des - r)]]).T

    return F, M
