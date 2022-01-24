

import numpy as np

mass = 0.18 # kg
g = 9.81 # m/s/s
I = np.array([[0.00025, 0, 2.55e-6],
              [0, 0.000232, 0],
              [2.55e-6, 0, 0.0003738]])


invI = np.linalg.inv(I)
arm_length = 0.086 # meter
height = 0.05
minF = 0.0
maxF = 2.0 * mass * g
L = arm_length
H = height
km = 1.5e-7 #motor constant
kf = 2e-7 #propeller drag constant
r = km / kf

#  [ F  ]         [ F1 ]
#  | M1 |  = A *  | F2 |
#  | M2 |         | F3 |
#  [ M3 ]         [ F4 ]
A = np.array([[ 1,  1,  1,  1],
              [ 0,  L,  0, -L],
              [-L,  0,  L,  0],
              [ r, -r,  r, -r]])

invA = np.linalg.inv(A)

body_frame = np.array([(3*L, 0, 0, 1),
                       (0, 3*L, 0, 1),
                       (-3*L, 0, 0, 1),
                       (0, -3*L, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, H, 1)])
