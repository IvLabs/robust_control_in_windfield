

import numpy as np
import scipy.integrate as integrate
from utils.quaternion import Quaternion
from utils.utils import RPYToRot, RotToQuat, RotToRPY
import model.params as params
import math as m
from windfield import Wind

class Quadcopter:
    """ Quadcopter class

    state  - 1 dimensional vector but used as 13 x 1. [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
             where [qw, qx, qy, qz] is quaternion and [p, q, r] are angular velocity [roll_dot, pitch_dot, yaw_dot]
    F      - 1 x 1, thrust output from controller
    M      - 3 x 1, moments output from controller
    params - system parameters struct, arm_length, g, mass, etc.
    """

    def __init__(self, pos, attitude):
        """ pos = [x,y,z] attitude = [roll,pitch,yaw]
            """
        self.state = np.zeros(13)
        self.roll, self.pitch, self.yaw = attitude
        roll, pitch, yaw = attitude
        rot    = RPYToRot(roll, pitch, yaw)   #Roll pitch yaw to Rotation matrix ZXY convention
        quat   = RotToQuat(rot)
        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = pos[2]
        self.state[3] = quat[0]
        self.state[4] = quat[1]
        self.state[5] = quat[2]
        self.state[6] = quat[3]
        self.accel = np.zeros(3)
        self.wind = Wind()

    def A(self):
        ''' Linear State-Space System Matrix'''
        A = np.zeros((12, 12))
        A[0, 6] = 1.
        A[1, 7] = 1.
        A[2, 8] = 1.
        A[6, 4] = params.g
        A[7, 3] = -params.g
        A[3, 9] = 1.
        A[4, 10] = 1.
        A[5, 11] = 1.
        return A

    def B(self):
        ''' Control input Matrix'''
        B = np.zeros((12, 4))
        B[9, 1] = 1/params.I[0,0]
        B[10, 2] = 1/params.I[1,1]
        B[11, 3] = 1/params.I[2,2]
        B[8, 0] = 1/params.mass
        return B

    def C(self):
        ''' Output Matrix'''
        C = np.eye(12)
        return C

    def D(self):
        ''' Feed-forward matrix'''
        D = np.zeros((12, 4))
        return D


    def world_frame(self):
        """ returns position of motors, quadcopter base centre of mass in a matrix
            where row is [x, y, z] column is m1 m2 m3 m4 origin h
            """
        origin = self.state[0:3]
        quat = Quaternion(self.state[3:7])
        rot = quat.as_rotation_matrix()
        rot = rot.T
        wHb = np.r_[np.c_[rot,origin], np.array([[0, 0, 0, 1]])]
        quadBodyFrame = params.body_frame.T
        quadWorldFrame = wHb.dot(quadBodyFrame)
        world_frame = quadWorldFrame[0:3]
        return world_frame

    def position(self):
        return self.state[0:3]

    def velocity(self):
        return self.state[7:10]

    def attitude(self):
        rot = Quaternion(self.state[3:7]).as_rotation_matrix()
        return RotToRPY(rot)

    def omega(self):
        return self.state[10:13]

    def state_dot(self, state, t, F, M,aerodynamic_forces):
        ''' generates derivative of states according to quadcopter equations of motion'''
 
        x, y, z, qw, qx, qy, qz, xdot, ydot, zdot, p, q, r = state
        quat = np.array([qw,qx,qy,qz])

        bRw = Quaternion(quat).as_rotation_matrix() # world to body rotation matrix
        wRb = bRw.T # orthogonal matrix inverse = transpose


        # acceleration - Newton's second law of motion

        accel = 1.0 / params.mass * (wRb.dot((np.array([[0, 0, F]]) + aerodynamic_forces).T)
                    - np.array([[0, 0, params.mass * params.g]]).T )
        self.accel = accel
        #print(accel)
        # angular velocity - using quternion
        # http://www.euclideanspace.com/physics/kinematics/angularvelocity/
        K_quat = 2.0; # this enforces the magnitude 1 constraint for the quaternion
        quaterror = 1.0 - (qw**2 + qx**2 + qy**2 + qz**2)
        qdot = (-1.0/2) * np.array([[0, -p, -q, -r],   # derivative of quaternion
                                    [p,  0, -r,  q],
                                    [q,  r,  0, -p],
                                    [r, -q,  p,  0]]).dot(quat) + K_quat * quaterror * quat

        # angular acceleration - Euler's equation of motion
        # https://en.wikipedia.org/wiki/Euler%27s_equations_(rigid_body_dynamics)
        omega = np.array([p,q,r])
        #print(M.shape)
        pqrdot = params.invI.dot(M.flatten() - np.cross(omega, params.I.dot(omega)))  # derivative of angular velocities
        state_dot = np.zeros(13)
        state_dot[0]  = xdot
        state_dot[1]  = ydot
        state_dot[2]  = zdot
        state_dot[3]  = qdot[0]
        state_dot[4]  = qdot[1]
        state_dot[5]  = qdot[2]
        state_dot[6]  = qdot[3]
        state_dot[7]  = accel[0]
        state_dot[8]  = accel[1]
        state_dot[9]  = accel[2]        
        state_dot[10] = pqrdot[0]
        state_dot[11] = pqrdot[1]
        state_dot[12] = pqrdot[2]

        return state_dot

    def update(self, dt, F, M):
        ''' updates quadcopter states according to given force and moments'''
        # limit thrust and Moment
        L = params.arm_length
        r = params.r
               
        prop_thrusts = params.invA.dot(np.r_[F, M])
        prop_thrusts_clamped = np.maximum(np.minimum(prop_thrusts, params.maxF/4), params.minF/4)
        omsq = np.array([prop_thrusts_clamped[0]/params.km, prop_thrusts_clamped[1]/params.km, prop_thrusts_clamped[2]/params.km, prop_thrusts_clamped[3]/params.km])
    
        F = np.sum(prop_thrusts_clamped)
        #F = F = np.sum(prop_thrusts_clamped)  # for clamped z thrust
       
        M = params.A[1:].dot(prop_thrusts_clamped)
        #M = params.A[1:].dot(prop_thrusts_clamped)   # for clamped moments
        vw,u,v = self.wind.windfield_gen(self.state[0],self.state[1])
        vw = 2*vw
        u = 2*u
        v = 2*v
        #print(np.linalg.norm(vw))
        #vw = np.array([1, 1, 1])
        vd = np.array([vw[0]-self.state[7], vw[1]-self.state[8], vw[2]-self.state[9]])
        sp = m.sqrt(vd[0]**2 + vd[1]**2 + vd[2]**2)  

        # Aerodynamic forces and moments
        Fx = 0.5 * 1.225 * 0.08 * sp * vd[0]
        Fy = 0.5 * 1.225 * 0.08 * sp * vd[1]
        Fz = 0.5 * 1.225 * 0.08 * sp * vd[2]
        aerodynamic_forces = np.array([[Fx,Fy,Fz]])
        #aerodynamic_forces = np.array([[0,0,0]])

        #Thrust reaction torques
        '''Mt = np.array([[params.km*(omsq[3]-omsq[1])*params.L, params.kf*(omsq[0]-omsq[2])*params.L, 0]]).T
        Mq = np.array([[0, 0, params.kf*(-omsq[0]+omsq[1]-omsq[2]+omsq[3])]]).T
        M_total = Mt + Mq + M'''
        self.state = integrate.odeint(self.state_dot, self.state, [0,dt], args = (F, M,aerodynamic_forces))[1]
        return prop_thrusts, u,v