#!/usr/bin/env python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import TwistStamped
from quadcopter.msg import SendActuator


rospy.init_node('testing',anonymous=True)
pub = rospy.Publisher('/desired/motor_speeds',SendActuator,queue_size=5)
rate = rospy.Rate(5)
motorspeed = SendActuator()
motorspeed.motor1 = 400
motorspeed.motor2 = 400
motorspeed.motor3 = 400
motorspeed.motor4 = 400
k = 0
while not rospy.is_shutdown():
    if k >=5:
        motorspeed.motor1 = 400
        motorspeed.motor2 = 400
        motorspeed.motor3 = 400
        motorspeed.motor4 = 400
    pub.publish(motorspeed)
    k = k + 1
    rate.sleep()