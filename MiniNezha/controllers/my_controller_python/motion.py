#!/usr/bin/env python3.7
# -*- coding: UTF-8 -*-from controller import Motor
from controller import InertialUnit
from controller import Gyro
from controller import Brake
import math
from PID_control import *

import math


class velocity_controller:

    def __init__(self, motors, panel, robot):
        self.Ev = 0.0
        self.panel = panel
        self.motors = motors
        self.factor1 = 1
        self.factor2 = 1
        # 平衡小车之家说还要乘0.6,我没乘
        # 角度
        self.pitch_Kp = 10.0  # 2.8  # 4 的时候平衡车，kp越大越稳
        self.pitch_Kd = 0.0  # 再大就会抖
        self.count = 0
        self.pitch_exp = 0
        self.cur_height = 0.43

        self.blance_u = 0.0
        self.blance_pid = PID_Controller(self.pitch_Kp, self.pitch_Kd)
        # 摆动角速度
        self.omgz_Kp = 2.0  # 2
        self.omgz_Kd = 0.0  # 再大一点就会抖

        self.omgz_u = 0.0
        self.omgz_pid = PID_Controller(self.omgz_Kp, self.omgz_Kd, 0.0)

        # body速度
        self.translation_Kp = 10000.0  # 8000  #
        self.translation_Kp1 = 0.00001  # 0.00008  # 这一项确定数量级
        self.translation_Ki = 0.0  #

        self.translation_u = 0.0
        self.translation_pid = PID_Controller(self.translation_Kp, 100000, self.translation_Ki)

        # 转弯控制
        self.rotation_u = 0.0

        self.rotation_Kp = 50.0
        self.rotation_Ki = 2.5
        self.rotation_Kd = 5.0

        self.rotation_pid = PID_Controller(self.rotation_Kp, self.rotation_Kd, self.rotation_Ki)

        self.theta3 = 0
        self.theta2 = 0
        self.theta1 = 0
        self.key = 0

        self.TIME_STEP = int(robot.getBasicTimeStep())
        self.robot = robot

    def calc_balance_angle_1(self, h):
        '''
        legs without mass
        '''
        theta3 = math.acos((51 * (-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                    1 / 2)) / 104)
        theta2 = math.pi - math.acos((-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                             1 / 2) / 2) - theta3
        theta1 = -math.pi / 2 + math.acos((-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                                  1 / 2) / 2)
        self.theta3, self.theta2, self.theta1 = theta3, theta2, theta1
        return theta1, theta2, theta3

    def setHeight(self, h):
        '''
        :h: -1 for extract
        '''
        if h == -1:
            self.motors[0].setPosition(0)
            self.motors[1].setPosition(0)
            self.motors[2].setPosition(0)
            self.motors[3].setPosition(0)
            self.cur_height = 0.45
        else:
            t1, t2, _ = self.calc_balance_angle_1(h)
            self.motors[0].setPosition(t1)
            self.motors[1].setPosition(t1)
            self.motors[2].setPosition(t2)
            self.motors[3].setPosition(t2)
            self.cur_height = h

    def setXVel(self, Ev):  # 注意：pitch方向和车方向相反，前倾为负

        if Ev == 0.0:
            self.pitch_exp = -0.007 + 0.07 * self.panel.bodyVel - 0.048 * (0.3 - self.cur_height)
        elif Ev > 0:
            self.pitch_exp = -0.007 + 0.05 * (self.panel.bodyVel - Ev) / Ev
        else:
            self.pitch_exp = -0.007 - 0.05 * (self.panel.bodyVel - Ev) / Ev
        # print("self.theta3",self.theta3)
        # print("self.pitch_exp",self.pitch_exp)
        if self.pitch_exp > self.theta3 / 10:
            self.pitch_exp = self.theta3 / 10
        elif self.pitch_exp < -0.1:
            self.pitch_exp = -0.1
        # print("self.pitch_exp2", self.pitch_exp)

        # 直立
        pitch_err = self.pitch_exp - self.panel.pitch
        self.blance_pid.feedback(pitch_err)
        self.blance_u = self.blance_pid.get_u()
        omgz_err = self.panel.omega_z
        self.omgz_pid.feedback(omgz_err)
        self.omgz_u = -self.omgz_pid.get_u()

        # 速度
        v_least = -self.panel.bodyVel + Ev  # 使用身体速度
        self.Ev *= 0.8
        self.Ev += v_least * 0.4
        translation_err = self.Ev
        self.translation_pid.feedback(translation_err)
        self.translation_u = self.translation_pid.get_u()

        self.motors[4].setTorque(
            -self.blance_u - self.omgz_u + self.translation_Kp1 * self.translation_u + 0.028 * self.rotation_u)
        self.motors[5].setTorque(
            -self.blance_u - self.omgz_u + self.translation_Kp1 * self.translation_u - 0.028 * self.rotation_u)

    def setAVel(self, key, vel):
        rotation_err = vel - self.panel.omega_y
        self.rotation_pid.feedback(rotation_err)
        if key == 65:
            self.rotation_u = self.rotation_pid.get_u()
            self.panel.rotation = 1
        elif key == 68:
            self.rotation_u = self.rotation_pid.get_u()
            self.panel.rotation = -1
        elif key == 70:
            self.rotation_u = 0.0
            self.panel.rotation = 0

    # def jump(self, robot, h=0.27):
    #     pre_velocity = self.panel.rightWheelVel
    #     # samplingPeriod = self.motors[2].getTorqueFeedbackSamplingPeriod() / 1000
    #     # print('sam:%3f' % samplingPeriod)
    #     # self.motors[2].enableTorqueFeedback(samplingPeriod)
    #     self.motors[2].enableTorqueFeedback(1)
    #     self.motors[0].enableTorqueFeedback(1)
    #     self.motors[2].setPosition(0)
    #     self.motors[3].setPosition(0)
    #     self.motors[0].setPosition(0)
    #     self.motors[1].setPosition(0)
    #     tor2 = self.motors[2].getTorqueFeedback()
    #     tor0 = self.motors[2].getTorqueFeedback()
    #     velocity2 = self.motors[2].getVelocity()
    #     velocity0 = self.motors[0].getVelocity()
    #     print('torque[2]:%3f' % tor2)
    #     print('Velocity[2]:%3f' % velocity2)
    #     print('torque[0]:%3f' % tor0)
    #     print('Velocity[0]:%3f' % velocity0)
    #
    #     # GPS版（项目临时）
    #     while 1:
    #         TIME_STEP = int(robot.getBasicTimeStep())
    #         robot.step(TIME_STEP)
    #         self.sensor_update()
    #         print("jump phase 1")
    #         if self.panel.gps_y > 0.48:  # 直立时gps0.488
    #             while 1:
    #                 print("jump phase 2")
    #                 TIME_STEP = int(robot.getBasicTimeStep())
    #                 robot.step(TIME_STEP)
    #                 self.sensor_update()
    #                 self.setHeight(h)
    #                 if self.panel.gps_y <= h + 0.06:
    #                     while 1:  # 为了保留结构暂时这么写，不要奇怪这个while break
    #                         print("jump phase 3")
    #                         TIME_STEP = int(robot.getBasicTimeStep())
    #                         robot.step(TIME_STEP)
    #                         self.sensor_update()
    #                         self.motors[4].setPosition(float('+inf'))
    #                         self.motors[5].setPosition(float('+inf'))
    #                         self.motors[4].setVelocity(pre_velocity)
    #                         self.motors[5].setVelocity(pre_velocity)
    #                         break
    #                         # if self.panel.gps_y > 0.49:
    #                         #     while 1:
    #                         #         print("jump phase 4")
    #                         #         TIME_STEP = int(robot.getBasicTimeStep() / 16)
    #                         #         robot.step(TIME_STEP)
    #                         #         self.panel.updateGPS()
    #                         #         if self.panel.gps_y <= 0.49:
    #                         #             # h = 0.44
    #                         #             # self.motors[4].setPosition(float('+inf'))
    #                         #             # self.motors[5].setPosition(float('+inf'))
    #                         #             # self.motors[4].setVelocity(0)
    #                         #             # self.motors[5].setVelocity(0)
    #                         #             # while 1:
    #                         #             #     print("jump phase 5")
    #                         #             #     TIME_STEP = int(robot.getBasicTimeStep() / 16)
    #                         #             #     robot.step(TIME_STEP)
    #                         #             #     self.panel.updateGPS()
    #                         #             #     if self.panel.gps_y <= h + 0.05:
    #                         #             #         h -= 0.01
    #                         #             #         self.setHeight(h)
    #                         #             #     if h <= 0.3:
    #                         #             #         break
    #                         #             break
    #                         #     break
    #                     break
    #             # self.setHeight(0.43)
    #             break
    #
    #     # TODO 下面为轮子空转版本 可以不使用GPS 但未完全成熟
    #     # last_v = 0
    #     # count = 0
    #     # while 1:
    #     #     # 在空中不采取行动
    #     #     TIME_STEP = int(robot.getBasicTimeStep())
    #     #     robot.step(TIME_STEP)
    #     #     panel.updateEncoder()
    #     #     panel.updateWheelVelocity()
    #     #     print('jumping rightWheelVel:', panel.rightWheelVel)
    #     #     # 轮子腾空后空转，转速先增后降，通过转速降来判断腾空后的落地过程（用GPS效果更好但是是流氓办法）
    #     #     if abs(last_v) > abs(panel.rightWheelVel):
    #     #         count += 1
    #     #         if count >= 5:
    #     #             break  # 若轮子速度连续下降，即进入平衡车状态
    #     #     last_v = panel.rightWheelVel
    #
    #     # if abs(panel.rightWheelVel) >= 5:
    #     #     while 1:
    #     #         TIME_STEP = int(robot.getBasicTimeStep())
    #     #         robot.step(TIME_STEP)
    #     #         panel.updateEncoder()
    #     #         panel.updateWheelVelocity()
    #     #         print('2::', panel.rightWheelVel)
    #     #         if abs(panel.rightWheelVel) < 5:
    #     #             break
    #     #     break

    # def jump(self, robot, desire_h=0.06):  # desire_h
    #     self.sensor_update()
    #     t0 = 0.2  # desire time
    #     m = 7.8  # body mass
    #     mb = 5  # total mass
    #     l0 = 0.23  # leg length
    #     g = 9.81
    #
    #     # pre_velocity = self.panel.rightWheelVel
    #     self.motors[2].enableTorqueFeedback(1)
    #     self.motors[0].enableTorqueFeedback(1)
    #     self.motors[0].setTorque(0)  # make base floating
    #     self.motors[1].setTorque(0)
    #     # self.motors[2].setPosition(0)
    #     # self.motors[3].setPosition(0)
    #     # tor2 = self.motors[2].getTorqueFeedback()
    #     # tor0 = self.motors[2].getTorqueFeedback()
    #     # velocity2 = self.motors[2].getVelocity()
    #     # velocity0 = self.motors[0].getVelocity()
    #     # print('torque[2]:%3f' % tor2)
    #     # print('Velocity[2]:%3f' % velocity2)
    #     # print('torque[0]:%3f' % tor0)
    #     # print('Velocity[0]:%3f' % velocity0)
    #
    #     count = 0
    #     TIME_STEP = int(robot.getBasicTimeStep())
    #     while 1:
    #         count += 1
    #         if count * TIME_STEP * 0.001 >= t0:  # counts discrete steps, to calculate integral
    #             break
    #         robot.step(TIME_STEP)
    #         self.sensor_update()
    #         theta = math.pi - self.panel.encoder[2]  # angle between wo legs
    #         torque = -((1 / t0 * math.sqrt(2 * desire_h / m)) + g) * l0 * mb * math.cos(theta / 2)  # calculate torque based on model
    #         self.motors[2].setTorque(torque)
    #         self.motors[3].setTorque(torque)
    #         # self.printInfo()
    #     # self.motors[2].setTorque(0)
    #     # self.motors[3].setTorque(0)
    #     self.sensor_update()
    #
    #     h_ref = self.panel.gps_y  # height, when jump starts
    #     print('h_ref :%3f' % h_ref)
    #     h_max = 0  # height of the top point
    #     while 1:
    #         robot.step(TIME_STEP)
    #         self.sensor_update()
    #         self.motors[2].setPosition(self.panel.encoder[2])  # lock keen motors, avoid passing min-angle
    #         self.motors[3].setPosition(self.panel.encoder[3])
    #         if h_max <= self.panel.gps_y:
    #             h_max = self.panel.gps_y
    #         else:
    #             print('break')
    #             break
    #     delta_h = h_max - h_ref  # max delta_height, should be compared with desire_h
    #     print('Actual height: %3f' % delta_h)

    def jump(self, robot, desire_h=0.3, a, b):  # desire_h
        self.sensor_update()
        t0 = 0.7  # desire time
        m = 7.8  # body mass
        mb = 5  # total mass
        l0 = 0.22  # leg length
        g = 9.81

        # pre_velocity = self.panel.rightWheelVel
        self.motors[2].enableTorqueFeedback(1)
        self.motors[0].enableTorqueFeedback(1)
        self.motors[0].setTorque(0)  # make base floating
        self.motors[1].setTorque(0)
        # self.motors[2].setPosition(0)
        # self.motors[3].setPosition(0)
        # tor2 = self.motors[2].getTorqueFeedback()
        # tor0 = self.motors[2].getTorqueFeedback()
        # velocity2 = self.motors[2].getVelocity()
        # velocity0 = self.motors[0].getVelocity()
        # print('torque[2]:%3f' % tor2)
        # print('Velocity[2]:%3f' % velocity2)
        # print('torque[0]:%3f' % tor0)
        # print('Velocity[0]:%3f' % velocity0)

        count = 0
        energy = 0
        last_theta = math.pi - self.panel.encoder[2]
        TIME_STEP = int(robot.getBasicTimeStep())
        while 1:
            count += 1
            t = count * TIME_STEP * 0.001
            # if count * TIME_STEP * 0.001 >= t0:  # counts discrete steps, to calculate integral
            #     break
            robot.step(TIME_STEP)
            self.sensor_update()
            theta = math.pi - self.panel.encoder[2]  # angle between wo legs

            # torque = -((1 / t0 * math.sqrt(2 * desire_h / m)) + g) * l0 * mb * math.cos(theta / 2)  # torque based on model
            # torque = -35 # constant
            # torque = a * t + b * t ^ 2 + c * t ^ 3 # poly function
            torque = a * desire_h / (1 + math.exp(-b * t))  # sigmoid function, a > 0, b > 0

            energy = energy + torque * math.fabs(theta - last_theta)
            last_theta = theta

            self.motors[2].setTorque(torque)
            self.motors[3].setTorque(torque)
            if self.panel.gps_v >= math.sqrt(desire_h * 2 * g) * 7.8 / 5.6:
                print(self.panel.gps_v)
                print(math.sqrt(desire_h * 2 * g) * 7.8 / 5.6)
                break
        loss_v = (self.panel.gps_v - math.sqrt(desire_h * 2 * g) * 7.8 / 5.6) ** 2
        loss = loss_v * 0.9 + energy * 0.1  #权重可修改

        # self.printInfo()
        # self.motors[2].setTorque(0)
        # self.motors[3].setTorque(0)
        self.sensor_update()
        h_ref = self.panel.gps_y  # height, when jump starts
        print('h_ref :%3f' % h_ref)
        h_max = 0  # height of the top point
        while 1:
            robot.step(TIME_STEP)
            self.sensor_update()
            self.motors[2].setPosition(self.panel.encoder[2])  # lock keen motors, avoid passing min-angle
            self.motors[3].setPosition(self.panel.encoder[3])
            if h_max <= self.panel.gps_y:
                h_max = self.panel.gps_y
            else:
                print('break')
                break
        delta_h = h_max - h_ref  # max delta_height, should be compared with desire_h
        print('Actual height: %3f' % delta_h)

    def checkPitch(self, angle_thr=30):
        '''check body pitch'''
        if abs(self.panel.pitch) <= angle_thr / 180 * math.pi:
            return True
        return False

    def checkAcc(self, acc_thr=0.1):
        '''check body acceleration'''
        if abs(self.panel.bodyAcce) < acc_thr:
            return True
        return False

    def checkVel(self, vel_thr=0.05):
        '''check body velocity'''
        if abs(self.panel.bodyVel) < vel_thr:
            return True
        return False

    def shutdown(self, brakes, height=0.25):
        self.setHeight(height)
        brakes[0].setDampingConstant(10000)
        brakes[1].setDampingConstant(10000)

    def restart(self, brakes, torque=13.5, height=0.25):
        '''full edition'''
        brakes[0].setDampingConstant(0)
        brakes[1].setDampingConstant(0)
        if self.panel.pitch >= 0:
            self.motors[4].setTorque(torque)
            self.motors[5].setTorque(torque)
        else:
            self.motors[4].setTorque(-torque)
            self.motors[5].setTorque(-torque)

        self.setHeight(height)

    def fall_recovery(self, restart_torque, brakes):
        # print("restart")
        self.restart(brakes, restart_torque, 0.25)
        if self.checkPitch(8):
            while (not self.checkAcc(0.1) and not self.checkVel(0.1)):
                # print("try balance")
                self.sensor_update()
                self.setXVel(0)
                self.robot.step(self.TIME_STEP)
            return True
        else:
            return False

    def showMsg(self, TIME):
        # file_handle = open('parameter.txt', mode='a')
        # file_handle.writelines([str(self.key),',',str(TIME),',',str(self.panel.pitch),',',str(self.panel.bodyVel), ',', str(self.panel.gps_v), '\n'])
        # file_handle.close()
        # print(TIME)
        print('-----------------')
        print("b_u: %.5f" % self.blance_u)
        print("t_u: %.5f" % (self.translation_Kp1 * self.translation_u))
        print("w_u: %.5f" % (self.wheel_Kp1 * self.wheel_u))
        print("pitch_err: %.3f" % self.pitch_err)
        print("pitch: %.5f" % self.panel.pitch)
        print("omgz_u: %.3f" % (0.01 * self.omgz_u))
        print("EV: %.3f" % (self.Ev))
        print("GPS_V: %.3f" % self.panel.gps_v)
        print("GPS_height: %.3f" % self.panel.gps_y)
        print("wheel_V: %.3f" % (self.panel.rightWheelVel * 0.05))
        print("body_V: %.5f" % self.panel.bodyVel)
        print("omega_y: %.5f" % self.panel.omega_y)
        print("omega_x: %.5f" % self.panel.omega_x)
        print("omega_z: %.5f" % self.panel.omega_z)
        print("期望速度： %.5f" % self.Ev)
        print("与期望速度差： %.5f" % (self.Ev - self.panel.rightWheelVel * 0.05))
        print("预期倾角：%.5f" % self.pitch_exp)
        print("Displacement: %.2f" % (self.panel.gps_dd))
        print("rWheelVel: %.5f" % (self.panel.rightWheelVel))
        print("rWheelVelSP: %.3f" % (self.panel.samplingPeriod))
        print('Angle1: %3f' % self.panel.encoder[0])
        print('Angle2: %3f' % self.panel.encoder[2])
        print('Angle3: %3f' % self.panel.encoder[4])
        print('-----------------')

    def keyboardControl(self, robot, key):
        self.key = key
        if key == 87:  # 'w' 前进
            self.setXVel(10)
            # print('forward')
        elif key == 83:  # 's' 后退
            self.setXVel(-10)
            # print('backward')
        elif key == 65:  # 'a' 左转
            self.setAVel(key, 0.6)
            # print('left')
        elif key == 68:  # 'd' 右转
            self.setAVel(key, -0.6)
            # print('right')
        elif key == 70:  # 'f' 停止旋转
            self.setAVel(key, 0.0)
        elif key == 315:  # '↑' 升高
            if self.cur_height < 0.43:
                self.cur_height += 0.01
            self.setHeight(self.cur_height)
        elif key == 317:  # '↓' 下降
            if self.cur_height > 0.2:
                self.cur_height -= 0.01
            self.setHeight(self.cur_height)
        elif key == 32:  # '空格' 跳跃  # 原key ==19
            self.jump(robot)
        elif key == 82:  # 'r' 重置
            self.robot.worldReload()
        else:
            # self.printInfo()
            self.setXVel(0.0)  # 0就是直立平衡；当前参数下，Ev=10时，实际速度仅为0.08

        rotation = self.panel.getRotation()
        if rotation == 1:
            self.setAVel(65, 0.6)
        elif rotation == -1:
            self.setAVel(68, -0.6)
        elif rotation == 0:
            self.setAVel(70, 0.0)
        # change robot position.
        # self.printInfo()

    def printInfo(self):
        print("GPS_height: %.3f" % self.panel.gps_y)
        print('Angle2: %3f' % self.panel.encoder[2])
        print('Torque: %3f' % self.motors[2].getTorqueFeedback())

    def sensor_update(self):
        # get sensors data
        self.panel.updateGPS()
        self.panel.updateIMU()
        self.panel.updateGyro()
        self.panel.updateTouch()
        self.panel.updateEncoder()
        self.panel.updateDirection()
        self.panel.updateWheelVelocity()
        self.panel.updateBodyVelocity(self.cur_height)
