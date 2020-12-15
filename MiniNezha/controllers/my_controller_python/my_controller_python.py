#!/usr/bin/env python
"""my_controller_python controller."""

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

for i in range(len(motor_names)):
    motors.append(robot.getMotor(motor_names[i]))
    encoders.append(robot.getPositionSensor(encoder_names[i]))
    encoders[i].enable(TIME_STEP)
    motors[i].setPosition(0)  # enable velocity control

# main loop
panel = panel(gps, gyro, imu, motors, encoders, TIME_STEP)
vel = velocity_controller(motors, panel)

vel.setHeight(0.4)

while robot.step(TIME_STEP) != -1:
    vel.showMsg()
    # get sensors data
    panel.updateGPS()
    panel.updateIMU()
    panel.updateGyro()
    panel.updateEncoder()
    panel.updateDirection()
    panel.updateWheelVelocity()
    panel.updateBodyVelocity(vel.cur_height)
    key = mKeyboard.getKey()  # 从键盘读取输入
    if vel.isFall():
        continue
    else:
        vel.keyboard_control(robot,key)

