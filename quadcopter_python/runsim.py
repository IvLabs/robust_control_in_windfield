
from utils.quadPlot import plot_quad_3d
from mpccontroller import MpcController
import utils.trajGen3D as trajGen3D
from model.quadcopter import Quadcopter
import model.params as params
import numpy as np
import cvxpy as cp
from lqrcontroller import LqrController
from plotting import GeneratePlots
import pidcontroller as pidcontrol
import geometriccontroller as gmcontrol
import smccontroller as smcontrol
import matplotlib.pyplot as plt
from math import sqrt
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import time as t

animation_frequency = 40
control_frequency = 160 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency




def attitudeControl(quad,time, desired_traj,flag,firstiter):
    ''' Control loop for generating suitable forces and moments to follow desired trajectory'''


    desired_state = np.c_[desired_traj[0],desired_traj[1],desired_traj[2]]

    indx = int(time*160)

    if flag == 1:      # pid controller
        F,M, des_rpy = pidcontrol.run(quad,desired_state,time)
    
        F = np.reshape(F,(1,1))
        rpy = quad.attitude()
        

    elif flag == 2:    # lqr controller

        if firstiter:
            controller = LqrController(quad)
            firstiter = False
        U = controller.run_lqr(desired_state,quad,time)
        F = U[0]
       
        M = np.array([U[1],U[2],U[3]])
        rpy = quad.attitude()
        des_rpy=np.array([0,0,0])
        
    elif flag == 3:    # mpc controller

        if firstiter:
            controller = MpcController(quad)
            firstiter = False
        #initialtime = t.time()
        problem = controller.run_mpc(desired_state,time)  # formulated optimization problem
        
        rpy = quad.attitude()

        # setting initial states in optimization problem
        controller.x0 = np.array([quad.state[0],quad.state[1],quad.state[2],rpy[0],rpy[1],\
            rpy[2],quad.state[7],quad.state[8],quad.state[9],quad.state[10],quad.state[11],quad.state[12]])
        controller.x_init.value = controller.x0

        # solving optimizaton problem
        problem.solve(solver=cp.OSQP)
        #print("problem formulation time = ",t.time()-initialtime)
        F = controller.u[0,0].value + params.mass*params.g

        M = np.array([controller.u[1,0].value, controller.u[2,0].value, controller.u[3,0].value])
        des_rpy = np.zeros(3)

    elif flag == 4:
        F,M = gmcontrol.run_gmc(quad,desired_state,time)
        #F = np.reshape(F,(1,1))
        rpy = quad.attitude()
        des_rpy = np.zeros(3)

    elif flag == 5:
        F,M = smcontrol.smc(quad,desired_state,time)
        F = np.reshape(F,(1,1))
        rpy = quad.attitude()  
        des_rpy = np.zeros(3)  

    
    
    motor_thrusts,u,v = quad.update(dt, F, M) # updating quadcopter states by giving generated F,M
    motor_thrusts = np.reshape(motor_thrusts,(4,1))



    return (desired_state[indx,0],desired_state[indx,1], desired_state[indx,2], des_rpy[0], des_rpy[1], des_rpy[2], quad.state[0], quad.state[1], quad.state[2] , rpy[0], rpy[1], rpy[2]),motor_thrusts,u,v



def main():
    global time

    # define parameters
    avgspeed = 2.0   # avg speed of quadrotor between start and end positions
    flag = 1         # 1 : PID, 2: LQR , 3: MPC, 4: GMC, 5: SMC
    wind_state = True     # presence of wind

    # don't change
    time = 0.0
    trajlength = 33.70  # total length of helix trajectory   
    iter = int((trajlength/avgspeed)*animation_frequency)
    plot_frames = np.zeros((3*iter,6))
    des_traj = np.zeros((iter,2))
    
    #############

    pos = (5,0,0)  # initial states
    attitude = (0,0,0)
    waypoints = trajGen3D.get_helix_waypoints(0, 9)   # waypoints for generating min. snap trajectory through them
    (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)
    des_trajectory = trajGen3D.trajasarray(time,dt,avgspeed,waypoints,coeff_x, coeff_y, coeff_z,iter)   # min. snap trajectory generation
    
    # quadcopter model object
    quadcopter = Quadcopter(pos, attitude, wind_state)

    # object for plotting
    genplots = GeneratePlots(iter)

    firstiter = True
    for i in range(int(iter)):
        print(i)
        for j in range(int(control_iterations)):
            
            des_state,motor_thrusts,u,v = attitudeControl(quadcopter,time,des_trajectory, flag,firstiter)
            genplots.update_plot_arrays(des_state,motor_thrusts,u,v,time,i,j)
            time += dt
            
        des_traj[i,:] = genplots.des_statexyz[4*i+3,0:2] 
        plot_frames[3*i:3*i+3,:] = quadcopter.world_frame()   
      
    dtwvalue, path = fastdtw(genplots.des_statexyz,genplots.curr_statexyz,dist=euclidean)
    print("dtwvalue = ",dtwvalue/(4*iter))

    genplots.plot_graphs()
    
    def animate(i):
        
        return plot_frames[3*i:3*i+3,:],genplots.u_arr[4*i],genplots.v_arr[4*i]
    
    
    plot_quad_3d(des_traj, animate,iter,wind_state)

    
if __name__ == "__main__":
    main()
