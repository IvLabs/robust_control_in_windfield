

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
import sys

from numpy.core.fromnumeric import repeat

history = np.zeros((1800,3))
count = 0
string = " "
def plot_quad_3d(des_traj, get_world_frame,itr):
    """
    get_world_frame is a function which return the "next" world frame to be drawn
    """

    global motors,pid,lqr,mpc,timelabel
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1])
    ax.plot([], [], [], '-', c='grey')
    ax.plot([], [], [], '-', c='red')
    ax.plot([], [], [], '-', c='blue', marker='o', markevery=2)
    ax.plot([], [], [], '.', c='red', markersize=4)
    timelabel=ax.text(0,0.5,string[0],fontsize=15)
    pid, = ax.plot([], [], '.', c='blue', markersize=2)
    lqr, = ax.plot([], [], '.', c='red', markersize=2)
    mpc, = ax.plot([], [], '.', c='green', markersize=2)
    motors, = ax.plot([], [], 'o', c='black')
    ax.plot(des_traj[:,0],des_traj[:,1],'.',c='black',markersize=2, markevery=4)
    set_limit((-6,6), (-6,6), (-2,2))
    
    an = animation.FuncAnimation(fig,
                                 anim_callback,
                                 fargs=(get_world_frame,),
                                 init_func=None,
                                 frames=itr, interval=25, blit=False, repeat=False)
    #f = r"/home/ayush/robustcontrol_in_wind/Python/results/1.5mspid.mp4"
    #an.save(f, dpi=80, writer='ffmpeg', fps=40)
   
    plt.show()

def set_limit(x, y, z):
    '''set axis limits'''
    ax = plt.gca()
    ax.set_xlim(x)
    ax.set_ylim(y)
    #ax.set_zlim(z)

def anim_callback(i, get_world_frame):
    '''callback function for animation'''
    frame = get_world_frame(i)
    set_frame(i,frame)

def set_frame(i,frame):
    '''plots quadcopter position in each frame'''
    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
    lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
    ax = plt.gca()
    string = 't={}'.format(round(i/40,2))
    timelabel.set_text(string)
    lines = ax.get_lines()
    
    for line, line_data in zip(lines[:3], lines_data):
        x, y, z = line_data
        line.set_data(x, y)
       # line.set_3d_properties(z)

    global history, count
    # plot history trajectory
    history[count] = frame[:,4].T
    if count < np.size(history, 0) - 1:
        count += 1
    xline = history[:count,0]
    yline = history[:count,1]
    
    '''if i <= 600:
        pid.set_data(xline[0:i], yline[0:i])
        lqr.set_data(0,0)
        mpc.set_data(0,0)
    if i >600 and i<=1200:
        pid.set_data(xline[0:600],yline[0:600])
        lqr.set_data(xline[601:i], yline[601:i])
        mpc.set_data(0,0)
    if i > 1200:
        pid.set_data(xline[0:600],yline[0:600])
        lqr.set_data(xline[601:1200],yline[601:1200])
        mpc.set_data(xline[1201:i], yline[1201:i])'''

    pid.set_data(xline,yline)

    
    motors.set_data(frame[0,[0,1,2,3]],frame[1,[0,1,2,3]])
    #lines[-1].set_3d_properties(zline)
    # ax.plot3D(xline, yline, zline, 'blue')
