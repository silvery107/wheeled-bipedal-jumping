#!/usr/bin/env python3.7
# -*- coding: UTF-8 -*-from controller import Motor
from utilities import *
from PID_control import *
import math
import sys


class velocity_controller:

    def __init__(self, motors, panel, robot):
        self.Ev = 0.0
        self.panel = panel
        self.motors = motors
        self.factor1 = 1
        self.factor2 = 1
        # 平衡小车之家说还要乘0.6,我没乘
        # 角度
        self.pitch_Kp = 10.0  # 2.8  # 4 的时候平衡车，kp越大越稳 10
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
        self.translation_Kp = 20000.0  # 8000  #10000
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

        self.imageCount = 0
        self.screenShotCount = 0

        self.isScreenShot = False
        self.isPointPos = False
        self.isPrint = False
        self.Bayes_Jump = False
        self.Model_Jump = False

        self.time = 0
        self.filename=''
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
        orig_angle = -0.013
        if Ev == 0.0:
            self.pitch_exp = orig_angle + 0.07 * self.panel.bodyVel - 0.048 * (0.3 - self.cur_height)
        elif Ev > 0:
            self.pitch_exp = orig_angle + 0.05 * (self.panel.bodyVel - Ev) / Ev
        else:
            self.pitch_exp = orig_angle - 0.05 * (self.panel.bodyVel - Ev) / Ev
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

    def savePointPos(self):
        # WEBOTS_HOME/projects/robots/neuronics/ipr/worlds/ipr_cube.wbt
        # \Webots\projects\samples\howto\worlds\supervisor_trail.wbt
        # https://cyberbotics.com/doc/guide/supervisor-programming?tab-language=python
        if self.isPointPos:
            TIME = self.time
            file_handle = open(self.filename, mode='a')
            file_handle.writelines([str(TIME),',',str(self.panel.WheelPos[1]-0.05),',',str(self.panel.WheelPos[0]),'\n'])
            file_handle.close()
            # file_handle = open('BodyHeight.txt', mode='a')
            # file_handle.writelines([str(TIME), ',', str(self.panel.BodyHeight[1]), '\n'])
            # file_handle.close()

    def screenShot(self, filetype, quality=100):
        if self.isScreenShot:
            if self.screenShotCount % 64 == 0:
                file_str = "../../screenshot/" + filetype + str(self.imageCount) + ".jpg"
                self.robot.exportImage(file_str, quality)
                self.imageCount += 1
            self.screenShotCount += 1
        self.savePointPos()

    def jump(self, params, desire_h=0.3):  # desire_h
        a = params["jump_a"]
        b = params["jump_b"]
        c = params["jump_c"]
        d = params["jump_d"]
        opt_vel = params["opt_vel"]
        t0 = 0.09  # desire time
        m = 7.8  # body mass
        mb = 5  # total mass
        l0 = 0.22  # leg length
        g = 9.81

        self.motors[0].setTorque(0.05)  # make base floating and no over rotating
        self.motors[1].setTorque(0.05)
        # self.motors[4].setTorque(0.05)
        # self.motors[5].setTorque(0.05)
        self.motors[2].enableTorqueFeedback(1)
        offSpeed = 0
        count = 0
        energy = 0
        last_theta = math.pi - self.panel.encoder[2]
        penalties = [0, 0, 0, 0, 0,
                     0]  # torque <=0, torque >=-35, theta <=0.7pi, theta >=0.1pi, energy <=mgh, pitch <=0.15pi

        # take off phase
        startTime = self.robot.getTime()
        h_ref = self.panel.WheelPos[1]  # height, when jump starts
        while 1:
            count += 1
            t = count * self.TIME_STEP * 0.001
            self.robot.step(self.TIME_STEP)
            self.sensor_update()
            self.screenShot("Jump")
            theta = math.pi - self.panel.encoder[2]  # angle between two legs

            if self.robot.getTime() > 3:
                # print("timeout 3 take-off")
                break

            if self.Bayes_Jump:
                torque = -(10 * a * t + 100 * b * t ** 2 + 1000 * c * t ** 3 + 10 * d)  # poly function
                if self.panel.supervisorBodyVel[1] >= opt_vel:
                    offSpeed = self.panel.supervisorBodyVel[1]
                    break
            elif self.Model_Jump:
                torque = -((1 / t0 * 7.8 / 5 * math.sqrt(2 * desire_h / m)) + g) * l0 * mb / math.cos(
                    theta / 2)  # torque based on model
                desire_v = math.sqrt(desire_h * 2 * g) * 7.8 / 5.6
                if self.panel.supervisorBodyVel[1] >= desire_v:
                    offSpeed = self.panel.supervisorBodyVel[1]
                    break
            else:
                torque = -35
                desire_v = math.sqrt(desire_h * 2 * g) * 7.8 / 5.6
                if self.panel.supervisorBodyVel[1] >= desire_v:
                    offSpeed = self.panel.supervisorBodyVel[1]
                    break

            # print(self.robot.getTime() - startTime)
            # if self.robot.getTime()-startTime>t0:
            #     offSpeed = self.panel.gps_v
            #     # print(self.panel.gps_v)
            #     # print(math.sqrt(desire_h * 2 * g) * 7.8 / 5.6)
            #     # print("t:", t)
            #     break

            # if self.panel.gps_v >= math.sqrt(desire_h * 2 * g) * 7.8 / 5.6:
            #     print(self.panel.gps_v)
            #     print(math.sqrt(desire_h * 2 * g) * 7.8 / 5.6)
            #     print("t:",t)
            #     break

            # torque = -a * desire_h / (1 + math.exp(-b * t))-c  # sigmoid function, a > 0, b > 0

            if torque > 0:
                penalties[0] += 0.1 * square_penalize(torque)
            if torque < -35:
                penalties[1] += 0.1 * square_penalize(-torque - 35)
            if theta > math.pi * 0.7:
                penalties[2] += 100 * square_penalize(theta - math.pi * 0.7)
            if theta < math.pi * 0.1:
                penalties[3] += 1000 * square_penalize(-theta + 0.1 * math.pi)
            if theta > math.pi or theta <= 0:
                break  
            
            energy += math.fabs(torque) * math.fabs(theta - last_theta)
            last_theta = theta

            self.motors[2].setTorque(torque)
            self.motors[3].setTorque(torque)
        print("t:", t)
        print("takeoff speed:", self.panel.supervisorBodyVel[1])
        h_max = -1  # height of the top point

        # flight phase
        while 1:
            if self.robot.getTime() > 4:
                # print("timeout 4 flight")
                break
            self.robot.step(self.TIME_STEP)
            self.sensor_update()
            self.screenShot("Jump")
            # lock_val = (self.panel.encoder[2] if self.panel.encoder[2] < 3.14 else 3)
            # self.motors[2].setPosition(lock_val)  # lock keen motors, avoid passing min-angle
            # self.motors[3].setPosition(lock_val)
            self.motors[2].setPosition(3 / 4 * math.pi)
            self.motors[3].setPosition(3 / 4 * math.pi)
            if h_max <= self.panel.WheelPos[1]:
                h_max = self.panel.WheelPos[1]
            else:
                break

        delta_h = h_max - h_ref  # max delta_height, should be compared with desire_h
        print("wheel delta h: %.3f " % delta_h, 'h_ref: ', h_ref, 'h_max: ', h_max)
        delta_w_h = (mb * offSpeed * 5 / 7.8 * offSpeed / 9.81 - 5 * delta_h) / 2
        # print('Actual delta height: %3f' % delta_h)
        # print('Actual wheel delta height: %3f' % delta_w_h)
        # print('h_ref :%3f' % h_ref)
        # print('Actual height: %3f' % delta_h)
        # print('Actual wheel height: %3f' % delta_w_h)
        # energy_baseline = m*g*desire_h
        # if energy > energy_baseline:
        #     penalties[4] = square_penalize(energy-energy_baseline)

        # loss_v = (self.panel.gps_v - math.sqrt(desire_h * 2 * g) * 7.8 / 5.6) ** 2
        loss_height = math.fabs(delta_h - desire_h)
        loss = loss_height * 1000 + energy

        # landing phase
        self.setHeight(0.3)
        while 1:
            self.robot.step(self.TIME_STEP)
            self.sensor_update()
            self.screenShot("Touch")
            if self.robot.getTime() > 5:
                # print("timeout 5 landing")
                break
            if self.panel.pitch > 0.15 * math.pi:
                penalties[5] += 10 * square_penalize(self.panel.pitch - 0.15 * math.pi)
            if self.panel.pitch > 0.25 * math.pi:
                break

            touch_F = self.panel.F[0][1]
            if touch_F > 1:
                print('touch_F %3f' % touch_F)
                break

        for penalty in penalties:
            loss += penalty
        print("loss_height: %.3f" % (loss_height * 1000))
        print("penalty:", penalties, ",loss:", loss, "energy:", energy)

        return loss

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
        '''squat down and brake wheels'''
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

    def fall_recovery(self, brakes):
        # print("restart")
        self.restart(brakes)
        if self.checkPitch(8):
            while (not self.checkAcc(0.1) and not self.checkVel(0.1)):
                # print("try balance")
                self.setXVel(0)
                self.robot.step(self.TIME_STEP)
                self.sensor_update()
                self.screenShot("Jump")
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

    def keyboardControl(self, key, param_dic):
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
            self.jump(param_dic)
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
        # self.printInfo()

    def printInfo(self):
        print("GPS_height: %.3f" % self.panel.gps_y)
        print('Angle2: %3f' % self.panel.encoder[2])
        print('Torque: %3f' % self.motors[2].getTorqueFeedback())

    def sensor_update(self):
        '''update sensors data'''
        self.panel.updateGPS()
        self.panel.updateIMU()
        self.panel.updateGyro()
        self.panel.updateTouch()
        self.panel.updateEncoder()
        self.panel.updateWheelPos()
        self.panel.updateDirection()
        self.panel.updateBodyHeight()
        self.panel.updateWheelVelocity()
        self.time = self.robot.getTime()
        self.panel.updateSupervisorBodyVel()
        self.panel.updateBodyVelocity(self.cur_height)

    def printX(self, string):
        if self.isPrint:
            print(string)
