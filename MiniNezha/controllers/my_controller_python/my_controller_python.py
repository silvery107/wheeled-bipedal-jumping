#!/usr/bin/env python
"""my_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import panel as panel

from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import Gyro
from controller import Keyboard
from PID_control import *
from motion import *
from panel import *
import os
import sys
import time
import math

# init robot
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
X = 0
Y = 1
Z = 2

# init sensors and drivers
gyro = robot.getGyro("gyro")
gyro.enable(TIME_STEP)
imu = robot.getInertialUnit("inertial_unit")
imu.enable(TIME_STEP)
gps = robot.getGPS("gps")
gps.enable(TIME_STEP)
mKeyboard = Keyboard()  # 初始化键盘读入类
mKeyboard.enable(TIME_STEP)  # 以mTimeStep为周期从键盘读取

encoders = []  # joint motor encoders
encoder_names = [
    "left_hip_encoder",
    "right_hip_encoder",
    "left_keen_encoder",
    "right_keen_encoder",
    "left_wheel_encoder",
    "right_wheel_encoder"
]

motors = []  # joint motors
motor_names = [
    "left_hip_motor",
    "right_hip_motor",
    "left_keen_motor",
    "right_keen_motor",  # 3
    "left_wheel_motor",
    "right_wheel_motor"
]

qs = []  # joint variables
for i in range(len(motor_names)):
    motors.append(robot.getMotor(motor_names[i]))
    encoders.append(robot.getPositionSensor(encoder_names[i]))
    encoders[i].enable(TIME_STEP)
    qs.append(encoders[i].getValue())
    motors[i].setPosition(0)  # enable velocity control

# main loop
panel = panel(gps, gyro, imu, motors, encoders, TIME_STEP)
vel = velocity_controller(motors, panel)

h = 0.35 #44
vel.setHeight(h)
flag = 0.01
while robot.step(TIME_STEP) != -1:
    # get sensors data
    panel.updateGPS()
    panel.updateIMU()
    panel.updateGyro()
    panel.updateEncoder()
    panel.updateDirection()
    panel.updateWheelVelocity()
    panel.updateBodyVelocity(h)

    vel.setXVel(0.0)  # 0就是直立平衡；当前参数下，Ev=10时，实际速度仅为0.08
    # vel.setAVel(0.0,0.0)
    key = 0  # 初始键盘读入默认为0
    key = mKeyboard.getKey()  # 从键盘读取输入
    if key == 87:  # 'w' 前进
        vel.setXVel(0.5)
    elif key == 83:  # 's' 后退
        vel.setXVel(-0.5)
    elif key == 65:  # 'a' 左转
        # vel.setAVel(r,angle)
        print('left')
    elif key == 68:  # 'd' 右转
        # vel.setAVel(r,angle)
        print('right')
    elif key == 315:  # '↑' 升高
        if h < 0.35:
            h += flag
        vel.setHeight(h)
    elif key == 317:  # '↓' 下降
        if h > 0.2:
            h += (-flag)
        vel.setHeight(h)
    elif key == 32: #'空格‘ 跳跃
        print('jump')

    # vel.setHeight(h)
    # vel.setXVel(10.0)

    # change robot position.
