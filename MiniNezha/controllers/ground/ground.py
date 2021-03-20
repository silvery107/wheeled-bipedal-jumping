#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""ground controller."""
from controller import Robot
from controller import TouchSensor

# init robot
robot = Robot()
# robot= Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

touch_sensors = []
touch_sensor_name = ["ground_touch_sensor"]

for idx, name in enumerate(touch_sensor_name):
    touch_sensors.append(robot.getTouchSensor(name))
    touch_sensors[idx].enable(TIME_STEP)


while robot.step(TIME_STEP) != -1:
    temp = touch_sensors[0].getValues()
    print("gound force: ",("%.3f, %.3f, %.3f") % (temp[0],temp[1],temp[2]))