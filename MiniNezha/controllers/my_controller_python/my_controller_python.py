#!/usr/bin/env python
"""my_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import panel as panel

from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import Gyro
from PID_control import *
from motion import *
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
    motors[i].setPosition(float('inf'))  # enable velocity control
    motors[i].setVelocity(0)

# main loop
panel = panel(gps, gyro, imu, motors, encoders, TIME_STEP)
vel = velocity_controller(motors, panel)

while robot.step(TIME_STEP) != -1:
    # get sensors data
    panel.updateGPS()
    panel.updateIMU()
    panel.updateGyro()
    panel.updateEncoder()
    panel.upadteDirection()
    panel.updateWheelVelocity()

    if panel.gps_y > 0.45:
        for i in range(4):
            motors[i].setTorque(0.01)  # free fall
    else:
        for i in range(0, 4):
            motors[i].setPosition(float('inf'))  # restore velocity control
            motors[i].setVelocity(0)  # lock leg motors

        vel.setXVel(0.0)  # 0就是直立平衡

    # change robot position
