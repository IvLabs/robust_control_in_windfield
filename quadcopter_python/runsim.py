
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
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import time as t

animation_frequency = 40
control_frequency = 160 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency




def attitudeControl(quad,time, desired_traj,flag,firstiter):
    ''' Control loop for generating suitable forces and moments to follow desired trajectory'''

    indx = int(time[0]*control_frequency)
    desired_state = np.c_[desired_traj[0],desired_traj[1],desired_traj[2]]

    indx = int(time[0]*160)

    if flag == 1:      # pid controller
        F,M, des_rpy = pidcontrol.run(quad,desired_state,time[0])
        #print("des yaw = ", des_rpy[2])

        F = np.reshape(F,(1,1))
        #print(F)
        rpy = quad.attitude()
        

    elif flag == 2:    # lqr controller

        if firstiter:
            controller = LqrController(quad)
            firstiter = False
        U = controller.run_lqr(desired_state,quad,time[0])
        F = U[0]
       
        M = np.array([U[1],U[2],U[3]])
        rpy = quad.attitude()
        des_rpy=np.array([0,0,0])
        
    elif flag == 3:    # mpc controller

        if firstiter:
            controller = MpcController(quad)
            firstiter = False
        #initialtime = t.time()
        problem = controller.run_mpc(desired_state,time[0])  # formulated optimization problem
        
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
        F,M = gmcontrol.run_gmc(quad,desired_state,time[0])
        #F = np.reshape(F,(1,1))
        rpy = quad.attitude()
        des_rpy = np.zeros(3)

    elif flag == 5:
        F,M = smcontrol.smc(quad,desired_state,time[0])
        F = np.reshape(F,(1,1))
        rpy = quad.attitude()  
        des_rpy = np.zeros(3)  

    #print(F.shape)

    motor_thrusts,u,v = quad.update(dt, F, M) # updating quadcopter states by giving generated F,M
    motor_thrusts = np.reshape(motor_thrusts,(4,1))

    time[0] += dt

    return (desired_state[indx,0],desired_state[indx,1], desired_state[indx,2], des_rpy[0], des_rpy[1], des_rpy[2], quad.state[0], quad.state[1], quad.state[2] , rpy[0], rpy[1], rpy[2]),motor_thrusts,u,v



def main():
    global time
    time = [0.0]
    #totaltime = 
    n = 1
    trajlength = 33.70  # total length of helix trajectory 
    avgspeed = 2.5   # avg speed of quadrotor between start and end positions
    iters = int((trajlength/avgspeed)*animation_frequency)


    iter = n*iters
    plot_frames = np.zeros((3*n*iter,6))
    des_traj = np.zeros((n*iter,2))
    des_statexyz = np.zeros((4*n*iter,3))
    curr_statexyz = np.zeros((4*n*iter,3))

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
    flag = 1
    u_arr = []
    v_arr = []
    des_x_arr = np.array([])
    x_arr = np.array([])
    des_y_arr = np.array([])
    y_arr = np.array([])
    des_z_arr = np.array([])
    z_arr = np.array([])
    des_r_arr = np.array([])
    r_arr = np.array([])
    des_p_arr = np.array([])
    p_arr = np.array([])
    des_yaw_arr = np.array([])
    yaw_arr = np.array([])
    time_arr = np.array([])
    time_array = np.array([])
    motor_thrusts_array = np.array([[0,0,0,0]])

    firstiter = True
    for i in range(int(n*iter)):
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
            #des_statexyz[4*i+j,:],curr_statexyz[4*i+j,:] = attitudeControl(quadcopter,time,des_trajectory, flag,firstiter)
            des_state,motor_thrusts,u,v = attitudeControl(quadcopter,time,des_trajectory, flag,firstiter)
            u_arr.append(u)
            v_arr.append(v) 
            motor_thrusts_array = np.r_[motor_thrusts_array,motor_thrusts.T]
            time_array = np.append(time_array,time[0])
            des_statexyz[4*i+j,:] = np.array([des_state[0],des_state[1],des_state[2]])
            curr_statexyz[4*i+j,:] = np.array([des_state[6],des_state[7],des_state[8]])
        des_traj[i,:] = np.array([des_state[0],des_state[1]])   
        plot_frames[3*i:3*i+3,:] = quadcopter.world_frame()
        time_arr = np.append(time_arr,time[0])   
        des_x_arr = np.append(des_x_arr,des_state[0])
        x_arr = np.append(x_arr,des_state[6])
        des_y_arr = np.append(des_y_arr,des_state[1])
        y_arr = np.append(y_arr,des_state[7])
        des_z_arr = np.append(des_z_arr,des_state[2])
        z_arr = np.append(z_arr,des_state[8])
        des_r_arr = np.append(des_r_arr,des_state[3])
        r_arr = np.append(r_arr,des_state[9])
        des_p_arr = np.append(des_p_arr,des_state[4])
        p_arr = np.append(p_arr,des_state[10])
        des_yaw_arr = np.append(des_yaw_arr,des_state[5])
        yaw_arr = np.append(yaw_arr,des_state[11])
        #print(des_x_arr)
        #print(time_arr)


    figure, axis = plt.subplots(2, 2)
    axis[0,0].plot(time_array[0:],motor_thrusts_array[1:,0])
    axis[0,1].plot(time_array[0:],motor_thrusts_array[1:,1])
    axis[1,0].plot(time_array[0:],motor_thrusts_array[1:,2])
    axis[1,1].plot(time_array[0:],motor_thrusts_array[1:,3])
    plt.show()
    dtwvalue, path = fastdtw(des_statexyz,curr_statexyz,dist=euclidean)
    print("dtwvalue = ",dtwvalue/(4*n*iter))
 
    # Initialise the subplot function using number of rows and columns
    figure, axis = plt.subplots(2, 2)
    
    # For Desired X
    #axis[0,0].plot(time_arr, des_x_arr-x_arr, color='r', label='Error X')
    axis[0,0].plot(time_arr, des_x_arr, color='r', label='Desired X')
    axis[0,0].plot(time_arr, x_arr, color='g', label='X')
    axis[0,0].set_title("X vs Time")
    #axis[0,0].plot(time_arr, des_x_arr - x_arr)
    #axis[0,0].set_title("Error X vs Time")
    
    # For Desired Y
    #axis[0,1].plot(time_arr, des_y_arr-y_arr, color='r', label='Error Y')
    axis[0,1].plot(time_arr, des_y_arr, color='r', label='Desired Y')
    axis[0,1].plot(time_arr, y_arr, color='g', label='Y')
    axis[0,1].set_title("Y vs Time")
    #axis[0,1].plot(time_arr, des_y_arr - y_arr)
    #axis[0,1].set_title("Error Y vs Time")

    # For Desired Z
    #axis[1,0].plot(time_arr, des_z_arr-z_arr, color='r', label='Error Z')
    axis[1,0].plot(time_arr, des_z_arr, color='r', label='Desired Z')
    axis[1,0].plot(time_arr, z_arr, color='g', label='Z')
    axis[1,0].set_title("Z vs Time")
    #axis[1,0].plot(time_arr, des_z_arr - z_arr)
    #axis[1,0].set_title("Error Z vs Time")    
    
    # Combine all the operations and display
    plt.show()   
    # Initialise the subplot function using number of rows and columns
    figure, axis = plt.subplots(2, 2)
    
    # For Desired Roll
    #axis[0,0].plot(time_arr, des_r_arr-r_arr, color='r', label='Error Roll')
    axis[0,0].plot(time_arr, des_r_arr, color='r', label='Desired Roll')
    axis[0,0].plot(time_arr, r_arr, color='g', label='Roll')
    axis[0,0].set_title("Desired Roll vs Time")
    #axis[0,0].plot(time_arr, des_r_arr - r_arr)
    #axis[0,0].set_title("Error Roll vs Time")    

    # For Desired Pitch
    #axis[0,1].plot(time_arr, des_p_arr-p_arr, color='r', label='Error Pitch')
    axis[0,1].plot(time_arr, des_p_arr, color='r', label='Desired Pitch')
    axis[0,1].plot(time_arr, p_arr, color='g', label='Pitch')
    axis[0,1].set_title("Desired Pitch vs Time")
    #axis[0,1].plot(time_arr, des_p_arr - p_arr)
    #axis[0,1].set_title("Error Pitch vs Time")    

    # For Desired Yaw
    #axis[1,0].plot(time_arr, des_yaw_arr-yaw_arr, color='r', label='Error Yaw')
    axis[1,0].plot(time_arr, des_yaw_arr, color='r', label='Desired Yaw')
    axis[1,0].plot(time_arr, yaw_arr, color='g',label='Yaw')
    axis[1,0].set_title("Desired Yaw vs Time")
    #axis[1,0].plot(time_arr, des_yaw_arr - yaw_arr)
    #axis[1,0].set_title("Error Yaw vs Time")    
        
    # Combine all the operations and display
    plt.show()

    def animate(i):
        
        return plot_frames[3*i:3*i+3,:],u_arr[4*i],v_arr[4*i]
    #print("cumulative position error = ",cpes)
    
    plot_quad_3d(des_traj, animate,n*iter)

    
if __name__ == "__main__":
    main()
