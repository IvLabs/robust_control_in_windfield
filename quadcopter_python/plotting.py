import numpy as np
import matplotlib.pyplot as plt

class GeneratePlots:
    def __init__(self,iter):
        self.des_statexyz = np.zeros((4*iter,3))
        self.curr_statexyz = np.zeros((4*iter,3))
        self.des_rpy = np.zeros((4*iter,3))
        self.rpy = np.zeros((4*iter,3))
        self.time_array = np.array([])
        self.motor_thrusts_array = np.array([[0,0,0,0]])
        self.u_arr = []
        self.v_arr = []

    def update_plot_arrays(self, des_state,motor_thrusts,u,v, time, i,j):
        self.u_arr.append(u)
        self.v_arr.append(v) 
        self.motor_thrusts_array = np.r_[self.motor_thrusts_array,motor_thrusts.T]
        self.time_array = np.append(self.time_array,time)
        self.des_statexyz[4*i+j,:] = np.array([des_state[0],des_state[1],des_state[2]])
        self.curr_statexyz[4*i+j,:] = np.array([des_state[6],des_state[7],des_state[8]])
        self.des_rpy[4*i+j,:] = np.array([des_state[3],des_state[4],des_state[5]])
        self.rpy[4*i+j,:] = np.array([des_state[9],des_state[10],des_state[11]])

    def plot_graphs(self):
        figure, axis = plt.subplots(2, 2)
        axis[0,0].plot(self.time_array[0:],self.motor_thrusts_array[1:,0])
        axis[0,1].plot(self.time_array[0:],self.motor_thrusts_array[1:,1])
        axis[1,0].plot(self.time_array[0:],self.motor_thrusts_array[1:,2])
        axis[1,1].plot(self.time_array[0:],self.motor_thrusts_array[1:,3])
        plt.show()

        figure, axis = plt.subplots(2, 2)
        axis[0,0].plot(self.time_array, self.des_statexyz[:,0], color='r', label='Desired X')
        axis[0,0].plot(self.time_array, self.curr_statexyz[:,0], color='g', label='X')
        axis[0,0].set_title("X vs Time")

        axis[0,1].plot(self.time_array, self.des_statexyz[:,1], color='r', label='Desired Y')
        axis[0,1].plot(self.time_array, self.curr_statexyz[:,1], color='g', label='Y')
        axis[0,1].set_title("Y vs Time")

        axis[1,0].plot(self.time_array, self.des_statexyz[:,2], color='r', label='Desired Z')
        axis[1,0].plot(self.time_array, self.curr_statexyz[:,2], color='g', label='Z')
        axis[1,0].set_title("Z vs Time")
        plt.show()

        figure, axis = plt.subplots(2, 2)
        axis[0,0].plot(self.time_array, self.des_rpy[:,0], color='r', label='Desired Roll')
        axis[0,0].plot(self.time_array, self.rpy[:,0], color='g', label='Roll')
        axis[0,0].set_title("Roll vs Time")

        axis[0,1].plot(self.time_array, self.des_rpy[:,1], color='r', label='Desired Pitch')
        axis[0,1].plot(self.time_array, self.rpy[:,1], color='g', label='Pitch')
        axis[0,1].set_title("Pitch vs Time")

        axis[1,0].plot(self.time_array, self.des_rpy[:,2], color='r', label='Desired Yaw')
        axis[1,0].plot(self.time_array, self.rpy[:,2], color='g', label='Yaw')
        axis[1,0].set_title("Yaw vs Time")
        plt.show()