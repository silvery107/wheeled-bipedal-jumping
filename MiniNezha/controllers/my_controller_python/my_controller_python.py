#!/usr/bin/env python3.7
# -*- coding: UTF-8 -*-
"""my_controller_python controller."""

from controller import Robot
from controller import Supervisor
from controller import Motor
from controller import InertialUnit
from controller import Gyro
from controller import Keyboard
from controller import Brake
from controller import TouchSensor

from PID_control import *
from motion import *
from panel import *
import os
import sys
import time
import math

# init robot
robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

# init sensors and drivers
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)

imu = robot.getDevice("inertial_unit")
imu.enable(TIME_STEP)

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

mKeyboard = Keyboard()
mKeyboard.enable(TIME_STEP)

touch_sensors = []
touch_sensor_name = ["left_touch_sensor", "right_touch_sensor"]

for idx, name in enumerate(touch_sensor_name):
    touch_sensors.append(robot.getDevice(name))
    touch_sensors[idx].enable(TIME_STEP)

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
    motors.append(robot.getDevice(motor_names[i]))
    encoders.append(motors[i].getPositionSensor())
    encoders[i].enable(TIME_STEP)
    motors[i].setPosition(0)  # enable velocity control

brakes = []
motors[4].enableTorqueFeedback(TIME_STEP)
motors[5].enableTorqueFeedback(TIME_STEP)
brakes.append(motors[4].getBrake())
brakes.append(motors[5].getBrake())

restart_torque = 0
metrics_dic = dict()
with open("./args.txt",'r') as args:
    param_dic = eval(args.read())
    restart_torque = param_dic["restart_torque"]

# main loop
panel = panel(gps, gyro, imu, motors, encoders, TIME_STEP, touch_sensors)
vel = velocity_controller(motors, panel, robot)

vel.setHeight(0.4)

fall_flag = False
restart_flag = False
restart_time0 = 0
restart_metrics = 99999
while robot.step(TIME_STEP) != -1:
    TIME = robot.getTime()
    # vel.showMsg(TIME)
    vel.sensor_update()
    key = mKeyboard.getKey()
    #
    # if TIME>5:
    #     break
        
    if fall_flag:
        if not restart_flag:
            restart_flag = vel.checkVel(0.005)
        if restart_flag:
            restart_time0 = robot.getTime()
            if vel.fall_recovery(restart_torque,brakes):
                restart_flag = False
                fall_flag = False
                restart_metrics = robot.getTime()-restart_time0
        else:
            # print("shutdown")
            vel.shutdown(brakes, 0.25)
            continue
    else:
        vel.keyboardControl(robot, key)
        fall_flag = not vel.checkPitch(30)

metrics_dic["restart_metrics"] = restart_metrics

with open("./metrics.txt",'w') as metrics:
    metrics.write(str(metrics_dic))

robot.simulationQuit(0)