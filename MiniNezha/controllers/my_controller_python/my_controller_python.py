#!/usr/bin/env python
"""my_controller_python controller."""

from controller import Robot
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

touch_sensors = []
touch_sensor_name = ["left_touch_sensor","right_touch_sensor"]
for idx,name in enumerate(touch_sensor_name):
    touch_sensors.append(robot.getTouchSensor(name))
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
    motors.append(robot.getMotor(motor_names[i]))
    encoders.append(motors[i].getPositionSensor())
    encoders[i].enable(TIME_STEP)
    motors[i].setPosition(0)  # enable velocity control

brakes = []
motors[4].enableTorqueFeedback(TIME_STEP)
motors[5].enableTorqueFeedback(TIME_STEP)
brakes.append(motors[4].getBrake())
brakes.append(motors[5].getBrake())

# main loop
panel = panel(gps, gyro, imu, motors, encoders, TIME_STEP)
vel = velocity_controller(motors, panel)

vel.setHeight(0.43)

fall_flag = False
restart_flag = False
while robot.step(TIME_STEP) != -1:
    TIME = robot.getTime()
    # vel.showMsg(TIME)
    vel.sensor_update()
    key = mKeyboard.getKey()  # 从键盘读取输入
    print(touch_sensors[0].getValues())

    if fall_flag:
        if not restart_flag:
            restart_flag = vel.checkVel(0.005)
        if restart_flag:
            print("restart")
            vel.restart(brakes,5,0.25)
            if vel.checkPitch(8):
                while (not vel.checkAcc(0.1) and not vel.checkVel(0.1)):
                    vel.sensor_update()
                    vel.setXVel(0)
                    print("try balance")
                    restart_flag = False
                    fall_flag = False
                    robot.step(TIME_STEP)
        else:
            print("shutdown")
            vel.shutdown(brakes,0.25)
            continue
    else:
        vel.keyboardControl(robot,key)
        fall_flag = not vel.checkPitch(30)

