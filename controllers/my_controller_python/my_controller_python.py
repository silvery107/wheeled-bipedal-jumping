#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""my_controller_python controller."""

from controller import Robot
from controller import Supervisor
from controller import Keyboard

from PID_control import *
from motion import *
from panel import *
from drawData import drawer
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
motors[2].enableTorqueFeedback(TIME_STEP)
motors[3].enableTorqueFeedback(TIME_STEP)
motors[4].enableTorqueFeedback(TIME_STEP)
motors[5].enableTorqueFeedback(TIME_STEP)
brakes.append(motors[4].getBrake())
brakes.append(motors[5].getBrake())

# main loop
panel = panel(gps, gyro, imu, motors, encoders, TIME_STEP, touch_sensors, robot)
vel = velocity_controller(motors, panel, robot)

vel.setHeight(0.3)
vel.setXVel(0.0)
# fall_flag = False
# restart_flag = False
jump_metrics = 9999
vel.Bayes_Jump = 1
vel.W_SLIP_Model_Jump = 0
vel.Time_Based_Jump = 0
height = 0.4
line = 2
dataDrawer = drawer(height)

isTraining = True
if not isTraining:
    '''
    valid csvName = poly-h=0.2, poly-h=0.3, poly-h=0.4,
                    sigmoid-h=0.2, sigmoid-h=0.3, sigmoid-h=0.35, sigmoid-h=0.4 , sigmoid-h=0.45, sigmoid-h=0.5
    !Change "height" above at the same time when changing the csvName!
    '''
    dataDrawer.changeArgs(height, line, csvName="../../records/sigmoid-h=0.45.csv")  # edit this only for Bayes_Jump
    dataDrawer.fileName = 'WheelPos' + str(dataDrawer.height) + '_'
    if (vel.Bayes_Jump):
        dataDrawer.fileName += str(dataDrawer.line)
        dataDrawer.fileName += '_Bayes'
    if (vel.W_SLIP_Model_Jump):
        dataDrawer.fileName += 'Slip'
    vel.filename = './dataset/' + dataDrawer.fileName + '.txt'
    dataDrawer.txtFileName = vel.filename

metrics_dic = dict()
with open("./args.txt", 'r') as args:
    param_dic = eval(args.read())

while robot.step(TIME_STEP) != -1:
    TIME = robot.getTime()
    vel.sensor_update()
    # vel.showMsg(TIME)
    # if fall_flag:
    #     if not restart_flag:
    #         restart_flag = vel.checkVel(0.005)
    #     if restart_flag:
    #         restart_time0 = robot.getTime()
    #         if vel.fall_recovery(brakes):
    #             restart_flag = False
    #             fall_flag = False
    #     else:
    #         print("shutdown")
    #         vel.shutdown(brakes, 0.25)
    #         continue
    # else:
    #     key = mKeyboard.getKey()
    #     vel.keyboardControl(key,param_dic)
    #     fall_flag = not vel.checkPitch(30)
    vel.isPrint = False
    vel.isPointPos = False
    vel.isScreenShot = False
    # if TIME > 3:
    #     break
    if 0 < TIME < 0.5:
        if vel.W_SLIP_Model_Jump:
            l_low = vel.obtain_delta_L_for_W_SLIP(height)
            vel.setHeight(l_low)
        else:
            vel.setHeight(0.2)
        vel.screenShot("1Start")
        vel.torque = motors[3].getTorqueFeedback()
    if 0.5 <= TIME < 1.5:
        vel.setXVel(3)
        vel.screenShot("1Start")
        vel.torque = motors[3].getTorqueFeedback()
    elif 1.5 <= TIME < 1.65:
        vel.setXVel(0)
        vel.screenShot("1Start")
        vel.torque = motors[3].getTorqueFeedback()
        if TIME ==1.55:
            dataDrawer.startIndex = vel.indexCount
    elif TIME >= 1.65 and jump_metrics == 9999:
        dataDrawer.startTorqueIndex = vel.indexCount
        jump_metrics = vel.jump(param_dic, height)
        dataDrawer.takeOffIndex = vel.takeOffIndex
        vel.screenShot("2Jump")
        # break
    else:
        vel.screenShot("4Land")
        key = mKeyboard.getKey()
        vel.keyboardControl(key, param_dic)
        vel.torque = motors[3].getTorqueFeedback()

    # key = mKeyboard.getKey()
    # vel.keyboardControl(key, param_dic)
    # vel.savePointPos()

metrics_dic["jump_metrics"] = jump_metrics

with open("./metrics.txt", 'w') as metrics:
    metrics.write(str(metrics_dic))

if isTraining:
    robot.simulationQuit(0)
else:
    dataDrawer.drawData()
