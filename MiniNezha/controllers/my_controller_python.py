#!/usr/bin/env python
"""my_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import Gyro
from PID_control import *
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
    "right_keen_motor",
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
Kp = 10
Kd = 10
blance_pid = PID_Controller(Kp, Kd)
gps_x, _, _ = gps.getValues()
x_last = gps_x
x_buffer = gps_x
count = 0
count_last = 0
blance_u = 0
while robot.step(TIME_STEP) != -1:
    # get sensors data
    gps_x, gps_y, gps_z = gps.getValues()  # gps_y is the height of robot
    roll, pitch, yaw = imu.getRollPitchYaw()  # pitch is the angle in rad w.r.t z-axis
    omega_x, omega_y, omega_z = gyro.getValues()

    # change robot position
    if gps_y > 0.45:
        for i in range(4):
            motors[i].setTorque(0.001)  # free fall
    else:
        for i in range(0, 4):
            motors[i].setPosition(float('inf'))  # restore velocity control
            motors[i].setVelocity(0)  # lock leg motors
        if x_last > gps_x:
            count = count + 1
        else:
            count = count - 1
        print(count)
        i = 0
        if count > 50:
            while i > 100:
                motors[4].setTorque(-0.1)
                motors[5].setTorque(0.1)
                i = i + 1
            count = count - 5
        elif count < -50:
            while i > 100:
                motors[4].setTorque(-0.05)
                motors[5].setTorque(-0.05)
                i = i + 1
            count = count + 5
        else:
            err = 0 - pitch
            blance_pid.feedback(err)
            blance_u = blance_pid.get_u()

            motors[4].setTorque(-blance_u)
            motors[5].setTorque(-blance_u)

    x_last = gps_x
    # count_last = count
    print("Y: %.3f" % (gps_y))
    print("X: %.3f" % (gps_x))
    print("Z: %.3f" % (gps_z))
    print("pitch: %.3f torque: %.3f" % (pitch, blance_u))
