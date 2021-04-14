#!/usr/bin/env python3.7
# -*- coding: UTF-8 -*-from controller import Motor
from utilities import *
from PID_control import *
import math
import sys


class velocity_controller:
    global t0, m, mb, l_leg, g, k, alpha_i, alpha_f, alpha_low, l0
    t0 = 0.09  # desire time
    m = 7.8  # total mass
    mb = 5.6  # body mass
    l_leg = 0.22  # leg length
    g = 9.81
    k = 1036.34
    alpha_i = 2.2822  # setHeight(0.4)
    alpha_f = 0
    # alpha_f = 0.9439  # setHeight(0.2)
      # setHeight(0.2)
    # alpha_low = 0
    l0 = 0.4

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
        self.Time_Based_Jump = False
        self.W_SLIP_Model_Jump = False

        self.torque = 0

        self.time = 0
        self.filename = ''

        self.indexCount = 0
        self.takeOffIndex = 0


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
        # self.printX("self.theta3",self.theta3)
        # self.printX("self.pitch_exp",self.pitch_exp)
        if self.pitch_exp > self.theta3 / 10:
            self.pitch_exp = self.theta3 / 10
        elif self.pitch_exp < -0.1:
            self.pitch_exp = -0.1
        # self.printX("self.pitch_exp2", self.pitch_exp)

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

    def savePointPos(self):
        # WEBOTS_HOME/projects/robots/neuronics/ipr/worlds/ipr_cube.wbt
        # \Webots\projects\samples\howto\worlds\supervisor_trail.wbt
        # https://cyberbotics.com/doc/guide/supervisor-programming?tab-language=python
        if self.isPointPos:
            TIME = self.time
            file_handle = open(self.filename, mode='a+')
            file_handle.writelines(
                [str(TIME), ',', str(self.panel.WheelPos[1] - 0.05), ',', str(self.panel.WheelPos[0]),',',str(self.torque), '\n'])
            file_handle.close()
            self.indexCount +=1
            # print("Motor torque: ", self.torque)
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

    def jump(self, params, desire_h):  # desire_h
        a = params["jump_a"]
        b = params["jump_b"]
        c = params["jump_c"]
        d = params["jump_d"]
        opt_vel = params["opt_vel"]

        self.motors[0].setTorque(0.05)  # make base floating and no over rotating
        self.motors[1].setTorque(0.05)
        # self.motors[4].setTorque(0.05)
        # self.motors[5].setTorque(0.05)
        self.motors[2].enableTorqueFeedback(1)
        offSpeed = 0
        count = 0
        energy = 0
        last_alpha = math.pi - self.panel.encoder[2]
        penalties = [0, 0, 0, 0, 0,
                     0]  # torque <=0, torque >=-35, alpha <=0.7pi, alpha >=0.1pi, energy <=mgh, pitch <=0.15pi

        # take off phase
        # startTime = self.robot.getTime()
        h_ref = self.panel.WheelPos[1]  # height, when jump starts
        init_torque = math.fabs(self.torque)
        t_start = math.log(35/init_torque-1)/(-10*a)
        while 1:
            count += 1
            t = count * self.TIME_STEP * 0.001
            self.robot.step(self.TIME_STEP)
            self.sensor_update()
            self.screenShot("Jump")
            alpha = math.pi - self.panel.encoder[2]  # angle between two legs

            if self.robot.getTime() > 3:
                # self.printX("timeout 3 take-off")
                break

            if self.Bayes_Jump:
                alpha_f = 0.9439
                # torque = -(10 * a * t + 100 * b * t ** 2 + 1000 * c * t ** 3 + 10 * d)  # poly function
                if b==0:
                    torque = -35/(1+math.exp(-10*a*(t+t_start)))
                else:
                    torque = -(10 * a * t + 100 * b * t ** 2 + 1000 * c * t ** 3 + 10 * d)  # poly function
                if self.panel.supervisorBodyVel[1] >= opt_vel:
                    offSpeed = self.panel.supervisorBodyVel[1]
                    break
            elif self.Time_Based_Jump:
                torque = -((1 / t0 * 7.8 / 5.6 * math.sqrt(2 * g * desire_h)) + g) * l_leg * mb * math.cos(
                    alpha / 2) / 2
                if torque <= -35:
                    torque = -35
                if t >= t0:
                    break
            elif self.W_SLIP_Model_Jump:
                alpha_f = 1.3
                if alpha >= alpha_i:
                    break
                damping = 0.14 * self.motors[2].getVelocity()
                torque = -(k * (l0 - 2 * l_leg * math.sin(alpha / 2)) * l0 * math.cos(alpha / 2) + damping) / 2
                if torque <= -35:
                    torque = -35
                # self.printX('rotate speed: %.3f' % self.motors[2].getVelocity())
                # self.printX('torque: %.3f' % torque)


            else:
                torque = -35
                desire_v = math.sqrt(desire_h * 2 * g) * m / mb
                if self.panel.supervisorBodyVel[1] >= desire_v:
                    offSpeed = self.panel.supervisorBodyVel[1]
                    break

            # self.printX(self.robot.getTime() - startTime)
            # if self.robot.getTime()-startTime>t0:
            #     offSpeed = self.panel.gps_v
            #     # self.printX(self.panel.gps_v)
            #     # self.printX(math.sqrt(desire_h * 2 * g) * 7.8 / 5.6)
            #     # self.printX("t:", t)
            #     break

            # if self.panel.gps_v >= math.sqrt(desire_h * 2 * g) * 7.8 / 5.6:
            #     self.printX(self.panel.gps_v)
            #     self.printX(math.sqrt(desire_h * 2 * g) * 7.8 / 5.6)
            #     self.printX("t:",t)
            #     break

            if torque > 0:
                penalties[0] += 0.1 * square_penalize(torque)
            if torque < -35:
                penalties[1] += 0.1 * square_penalize(-torque - 35)
            if alpha > math.pi * 0.7:
                penalties[2] += 100 * square_penalize(alpha - math.pi * 0.7)
            if alpha < math.pi * 0.1:
                penalties[3] += 1000 * square_penalize(-alpha + 0.1 * math.pi)
            if alpha > math.pi or alpha <= 0:
                break

            if torque <= -35:
                torque = -35
            energy += math.fabs(torque) * math.fabs(alpha - last_alpha)
            last_alpha = alpha

            self.torque = torque
            self.motors[2].setTorque(torque)
            self.motors[3].setTorque(torque)
            self.printX('Motor torque: %.3f' % self.motors[3].getTorqueFeedback())
        self.printX("t:", t)
        self.printX("takeoff speed:", self.panel.supervisorBodyVel[1])
        self.takeOffIndex = self.indexCount
        h_max = -1  # height of the top point

        # flight phase
        while 1:
            if self.robot.getTime() > 4:
                # self.printX("timeout 4 flight")
                break
            self.robot.step(self.TIME_STEP)
            self.sensor_update()
            self.screenShot("Jump")
            self.torque = self.motors[2].getTorqueFeedback()
            # lock_val = (self.panel.encoder[2] if self.panel.encoder[2] < 3.14 else 3)
            # self.motors[2].setPosition(lock_val)  # lock keen motors, avoid passing min-angle
            # self.motors[3].setPosition(lock_val)
            self.motors[2].setPosition(math.pi - alpha_f)
            self.motors[3].setPosition(math.pi - alpha_f)
            self.printX('knee angle: ',(math.pi - self.panel.encoder[2])-alpha_f)
            if h_max <= self.panel.WheelPos[1]:
                h_max = self.panel.WheelPos[1]
            else:
                break

        delta_h = h_max - h_ref  # max delta_height, should be compared with desire_h
        print("wheel delta h: %.5f " % delta_h, 'h_ref: ', h_ref, 'h_max: ', h_max)
        print("highest point: %.5f" % self.panel.WheelPos[0])
        delta_w_h = (mb * offSpeed * 5 / 7.8 * offSpeed / 9.81 - 5 * delta_h) / 2
        # self.printX('Actual delta height: %3f' % delta_h)
        # self.printX('Actual wheel delta height: %3f' % delta_w_h)
        # self.printX('h_ref :%3f' % h_ref)
        # self.printX('Actual height: %3f' % delta_h)
        # self.printX('Actual wheel height: %3f' % delta_w_h)

        # loss_v = (self.panel.gps_v - math.sqrt(desire_h * 2 * g) * 7.8 / 5.6) ** 2
        loss_height = math.fabs(delta_h - desire_h)
        loss = loss_height * 2000 + energy

        # landing phase
        self.setHeight(0.3)
        while 1:
            self.robot.step(self.TIME_STEP)
            self.sensor_update()
            self.screenShot("Touch")
            line_y = self.panel.BodyHeight[1]-self.panel.WheelPos[1]
            line_x = self.panel.BodyHeight[0]-self.panel.WheelPos[0]
            line_angle = math.atan2(line_y,line_x)

            if self.robot.getTime() > 5:
                # self.printX("timeout 5 landing")
                break
            if self.panel.pitch > 0.15 * math.pi:
                penalties[5] += 10 * square_penalize(self.panel.pitch - 0.15 * math.pi)
            if self.panel.pitch > 0.25 * math.pi:
                break
            if line_angle > 0.55*math.pi:
                penalties[4] += 10 * square_penalize(line_angle-0.55*math.pi)
            if line_angle > 0.65*math.pi:
                break
            touch_F = self.panel.F[0][1]
            if touch_F > 1:
                self.printX('touch_F %3f' % touch_F)
                break

        for penalty in penalties:
            loss += penalty
        self.printX("loss_height: %.3f" % (loss_height * 1000))
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
        # self.printX("restart")
        self.restart(brakes)
        if self.checkPitch(8):
            while (not self.checkAcc(0.1) and not self.checkVel(0.1)):
                # self.printX("try balance")
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
        # self.printX(TIME)
        self.printX('-----------------')
        self.printX("b_u: %.5f" % self.blance_u)
        self.printX("t_u: %.5f" % (self.translation_Kp1 * self.translation_u))
        self.printX("w_u: %.5f" % (self.wheel_Kp1 * self.wheel_u))
        self.printX("pitch_err: %.3f" % self.pitch_err)
        self.printX("pitch: %.5f" % self.panel.pitch)
        self.printX("omgz_u: %.3f" % (0.01 * self.omgz_u))
        self.printX("EV: %.3f" % (self.Ev))
        self.printX("GPS_V: %.3f" % self.panel.gps_v)
        self.printX("GPS_height: %.3f" % self.panel.gps_y)
        self.printX("wheel_V: %.3f" % (self.panel.rightWheelVel * 0.05))
        self.printX("body_V: %.5f" % self.panel.bodyVel)
        self.printX("omega_y: %.5f" % self.panel.omega_y)
        self.printX("omega_x: %.5f" % self.panel.omega_x)
        self.printX("omega_z: %.5f" % self.panel.omega_z)
        self.printX("期望速度： %.5f" % self.Ev)
        self.printX("与期望速度差： %.5f" % (self.Ev - self.panel.rightWheelVel * 0.05))
        self.printX("预期倾角：%.5f" % self.pitch_exp)
        self.printX("Displacement: %.2f" % (self.panel.gps_dd))
        self.printX("rWheelVel: %.5f" % (self.panel.rightWheelVel))
        self.printX("rWheelVelSP: %.3f" % (self.panel.samplingPeriod))
        self.printX('Angle1: %3f' % self.panel.encoder[0])
        self.printX('Angle2: %3f' % self.panel.encoder[2])
        self.printX('Angle3: %3f' % self.panel.encoder[4])
        self.printX('-----------------')

    def keyboardControl(self, key, param_dic):
        self.key = key
        if key == 87:  # 'w' 前进
            self.setXVel(10)
            # self.printX('forward')
        elif key == 83:  # 's' 后退
            self.setXVel(-10)
            # self.printX('backward')
        elif key == 65:  # 'a' 左转
            self.setAVel(key, 0.6)
            # self.printX('left')
        elif key == 68:  # 'd' 右转
            self.setAVel(key, -0.6)
            # self.printX('right')
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
        self.printX("GPS_height: %.3f" % self.panel.gps_y)
        self.printX('Angle alpha: %3f' % (math.pi - self.panel.encoder[2]))
        self.printX('Torque: %3f' % self.motors[2].getTorqueFeedback())

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

    def printX(self, string,a='',b='',c='',d='',e=''):
        if self.isPrint:
            print(string,a,b,c,d,e)

    def obtain_delta_L_for_W_SLIP(self, desire_h):
        mb = 5.7
        alpha_f=1.3
        delta_L = math.sqrt(
            (desire_h - 2 * l_leg * (
                    math.sin(alpha_i / 2) - math.sin(alpha_f / 2)) * mb / m) * 2 * g * m * m / mb / k)
        l_low = l0 - delta_L
        # alpha_low = 2 * math.asin(l_low / (2 * l_leg))
        return l_low
