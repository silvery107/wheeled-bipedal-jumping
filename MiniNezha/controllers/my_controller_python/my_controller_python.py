#!/home/silvery/anaconda3/envs/webots/bin/python
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
# restart_time0 = 0
# restart_metrics = 99999
jump_metrics = 9999


isTraining = Fasle
if not isTraining:
    dataDrawer = drawer()
    dataDrawer.changeArgs(0.2, 4)
    dataDrawer.fileName = 'WheelPos' + str(dataDrawer.height) + '_' + str(dataDrawer.line)
    vel.filename = './dataset/' + dataDrawer.fileName + '.txt'
    dataDrawer.txtFileName= vel.filename

metrics_dic = dict()
with open("./args.txt", 'r') as args:
    param_dic = eval(args.read())

while robot.step(TIME_STEP) != -1:
    TIME = robot.getTime()
    # vel.showMsg(TIME)
    vel.sensor_update()
    # if fall_flag:
    #     if not restart_flag:
    #         restart_flag = vel.checkVel(0.005)
    #     if restart_flag:
    #         restart_time0 = robot.getTime()
    #         if vel.fall_recovery(brakes):
    #             restart_flag = False
    #             fall_flag = False
    #             restart_metrics = robot.getTime()-restart_time0
    #     else:
    #         # print("shutdown")
    #         vel.shutdown(brakes, 0.25)
    #         continue
    # else:
    #     key = mKeyboard.getKey()
    #     vel.keyboardControl(key,param_dic)
    #     fall_flag = not vel.checkPitch(30)
    vel.isPrint = False
    vel.isPointPos = False
    vel.isScreenShot = False
    vel.Bayes_Jump = 0
    vel.Model_Jump = 1
    if TIME > 5:
        break
    if 0 < TIME < 0.5:
        vel.setHeight(0.2)
        vel.screenShot("Start")
    if 0.5 <= TIME < 1.5:
        vel.setXVel(3)
        vel.screenShot("Start")
    elif 1.5 <= TIME < 1.6:
        vel.setXVel(0)
        vel.screenShot("Start")
    elif TIME >= 1.6 and jump_metrics == 9999:
        jump_metrics = vel.jump(param_dic, 0.2)
        vel.screenShot("Jump")
        break
    else:
        vel.screenShot("Land")
        key = mKeyboard.getKey()
        vel.keyboardControl(key, param_dic)
    # key = mKeyboard.getKey()
    # vel.keyboardControl(key,param_dic)
    # vel.savePointPos()

# metrics_dic["restart_metrics"] = restart_metrics
metrics_dic["jump_metrics"] = jump_metrics

with open("./metrics.txt", 'w') as metrics:
    metrics.write(str(metrics_dic))

# dataDrawer.drawData()

robot.simulationQuit(0)
