

from control import *
from utils.quadPlot import plot_quad_3d
from mpccontroller import MpcController
import utils.trajGen3D as trajGen3D
from model.quadcopter import Quadcopter
import model.params as params
import numpy as np
import cvxpy as cp
from lqrcontroller import LqrController
import pidcontroller as pidcontrol
import geometriccontroller as gmcontrol
import smccontroller as smcontrol
import matplotlib.pyplot as plt
from math import sqrt

animation_frequency = 40
control_frequency = 160 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
cpes = 0


def attitudeControl(quad,time, desired_traj,flag,firstiter):
    ''' Control loop for generating suitable forces and moments to follow desired trajectory'''

    global cpes
    indx = int(time[0]*control_frequency)
    desired_state = np.c_[desired_traj[0],desired_traj[1],desired_traj[2]]

    indx = int(time[0]*160)

    if flag == 1:      # pid controller
        F,M = pidcontrol.run(quad,desired_state,time[0])
        
        F = np.reshape(F,(1,1))
        

    elif flag == 2:    # lqr controller

        if firstiter:
            controller = LqrController(quad)
            firstiter = False
        U = controller.run_lqr(desired_state,quad,time[0])
        F = U[0]
       
        M = np.array([U[1],U[2],U[3]])
        
    elif flag == 3:    # mpc controller

        if firstiter:
            controller = MpcController(quad)
            firstiter = False
        problem = controller.run_mpc(desired_state,time[0])  # formulated optimization problem
        rpy = quad.attitude()

        # setting initial states in optimization problem
        controller.x0 = np.array([quad.state[0],quad.state[1],quad.state[2],rpy[0],rpy[1],\
            rpy[2],quad.state[7],quad.state[8],quad.state[9],quad.state[10],quad.state[11],quad.state[12]])
        controller.x_init.value = controller.x0

        # solving optimizaton problem
        problem.solve(solver=cp.OSQP, warm_start=True)
        F = controller.u[0,0].value
        M = np.array([controller.u[1, 0].value, controller.u[2, 0].value, controller.u[3, 0].value])

    elif flag == 4:
        F,M = gmcontrol.run_gmc(quad,desired_state,time[0])
        F = np.reshape(F,(1,1))

    elif flag == 5:
        F,M = smcontrol.smc(quad,desired_state,time[0])
        F = np.reshape(F,(1,1))



    quad.update(dt, F, M) # updating quadcopter states by giving generated F,M

    avgspeed = sqrt(quad.state[7]**2 + quad.state[8]**2 + quad.state[9]**2)
    #print(avgspeed)
    cpe_dt = np.linalg.norm(np.array([desired_state[indx,0]-quad.state[0],desired_state[indx,1]-quad.state[1],desired_state[indx,2]-quad.state[2]]))
    cpes = cpes + cpe_dt    # cumulative position error 
    time[0] += dt

    return desired_state[indx,0],desired_state[indx,1]



def main():
    global time
    time = [0.0]
    #totaltime = 
    n = 1
    trajlength = 33.70  # total length of helix trajectory 
    avgspeed = 1.5   # avg speed of quadrotor between start and end positions
    iters = int((trajlength/avgspeed)*animation_frequency)


    iter = n*iters
    plot_frames = np.zeros((3*n*iter,6))
    des_traj = np.zeros((n*iter,2))

    trajswitch = 0    

    if trajswitch==0:
        pos = (5,0,0)  
        attitude = (0,0,0)
        waypoints = trajGen3D.get_helix_waypoints(0, 9)
    elif trajswitch==1:
        pos = (0,0,0)  
        attitude = (0,0,0)
        waypoints = trajGen3D.get_lemniscate_waypoints(0,50)
    else:
        pos = (0,-0.7,0)  
        attitude = (0,0,0)
        waypoints = np.array([[0,-0.7,0],[0.7,0,0],[0,0.7,0],[-0.7,0,0],[0,-0.7,0]])       

    # quadcopter model object
    quadcopter = Quadcopter(pos, attitude)

    # generating coefficients for minimum snap trajectory through generated waypoints
    (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)

    des_trajectory = trajGen3D.trajasarray(time[0],dt,avgspeed,waypoints,coeff_x, coeff_y, coeff_z,iter)

    '''desposition = des_trajectory[0]
    desposition_1 =  np.delete(desposition,0,axis=0)
    desposition_1 =  np.r_[desposition_1,np.reshape(desposition[-1,:],(1,3))]
    del_vector =  desposition_1 - desposition
    del_dist = np.linalg.norm(del_vector,axis=1)
    total_distance = np.sum(del_dist)
    print(total_distance)'''

    # flag for controller
    flag = 3 

    firstiter = True
    for i in range(n*iter):
        if i >= int(iter) and flag == 1:
            time[0] = 0.0
            if trajswitch==0:
                pos = (5,0,0)  
                attitude = (0,0,0)
            else:
                pos = (0,0,0)  
                attitude = (0,0,0)

            # quadcopter model object
            quadcopter = Quadcopter(pos, attitude)
            flag = 2
            firstiter = True
    
        elif i >= int(2*iter) and flag == 2:
            time[0] = 0.0
            if trajswitch==0:
                pos = (5,0,0)  
                attitude = (0,0,0)
            else:
                pos = (0,0,0)  
                attitude = (0,0,0)

            # quadcopter model object
            quadcopter = Quadcopter(pos, attitude)
            flag = 3
            firstiter = True

        elif i >= int(3*iter) and flag == 3:
            time[0] = 0.0
            if trajswitch==0:
                pos = (5,0,0)  
                attitude = (0,0,0)
            else:
                pos = (0,0,0)  
                attitude = (0,0,0)

            # quadcopter model object
            quadcopter = Quadcopter(pos, attitude)
            flag = 4
            firstiter = True

        
        print(i)
        for j in range(int(control_iterations)):
            des_state = attitudeControl(quadcopter,time,des_trajectory, flag,firstiter)
               
        des_traj[i,:] = np.array([des_state[0],des_state[1]])   
        plot_frames[3*i:3*i+3,:] = quadcopter.world_frame()

       
    def animate(i):
        
        return plot_frames[3*i:3*i+3,:]
    print("cumulative position error = ",cpes)
    
    plot_quad_3d(des_traj, animate,n*iter)

    
if __name__ == "__main__":
    main()
