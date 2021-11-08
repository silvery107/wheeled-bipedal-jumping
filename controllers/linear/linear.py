#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""linear controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
motors = []  # joint motors
encoders = []
motors.append(robot.getMotor("linear motor"))
encoders.append(robot.getPositionSensor("position sensor"))
encoders[0].enable(TIME_STEP)


##TODO 你只用改下面的
pos = 0
add = 0.3
while robot.step(TIME_STEP) != -1:
    
    #执行位置
    motors[0].setPosition(pos)
    #到达边界就回调
    if pos >= 6:
        add = -0.3
    elif pos <= -6:
        add = 0.3

    #确定到达上一个pos位置后，就定下一个pos
    if encoders[0].getValue() == pos:
        pos += add

# Enter here exit cleanup code.
