from controller import Motor
from controller import InertialUnit
from controller import Gyro
import numpy as np
from PID_control import *

import math


class velocity_controller:

    def __init__(self, motors, panel):
        self.Ev = 0.0
        self.panel = panel
        self.motors = motors
        self.factor1 = 1
        self.factor2 = 1
        # 平衡小车之家说还要乘0.6,我没乘
        # 角度
        self.pitch_Kp = 2.8  # 4 的时候平衡车，kp越大越稳
        self.pitch_Kd = 0.0  # 再大就会抖
        self.count = 0
        self.blance_u = 0.0
        self.blance_pid = PID_Controller(self.pitch_Kp, self.pitch_Kd)
        # 摆动角速度
        self.omgz_Kp = 2
        self.omgz_Kd = 0.0  # 再大一点就会抖
        self.omgz_u = 0.0
        self.omgz_pid = PID_Controller(self.omgz_Kp, self.omgz_Kd, 0.0)

        # body速度
        self.translation_Kp = 8000  #
        self.translation_Kp1 = 0.00008  # 这一项确定数量级
        self.translation_Ki = 10.0  #

        self.translation_u = 0.0
        self.translation_pid = PID_Controller(self.translation_Kp, 10000, self.translation_Ki)

        # 轮子速度
        self.wheel_Kp = 50
        self.wheel_Kp1 = 0.0015
        self.wheel_Ki = 10

        self.wheel_u = 0.0
        self.wheel_pid = PID_Controller(self.wheel_Kp, 20, self.wheel_Ki)

    def calc_balance_angle_1(self, h):
        '''
        legs without mass
        '''
        theta3 = np.arccos((51 * (-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                    1 / 2)) / 104)
        theta2 = np.pi - np.arccos((-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                           1 / 2) / 2) - theta3;
        theta1 = -np.pi / 2 + np.arccos((-(1081600 * h * (
                (132625 * h) / 103 - (11 * ((70331040000 * h ** 2) / 1283689 + 4) ** (1 / 2)) / 2)) / 12463) ** (
                                                1 / 2) / 2)
        return theta1, theta2, theta3

    def setHeight(self, h):
        t1, t2, t3 = self.calc_balance_angle_1(h)

        self.motors[0].setPosition(t1)
        self.motors[1].setPosition(t1)
        self.motors[2].setPosition(t2)
        self.motors[3].setPosition(t2)

    def printAngle(self, h):
        t1, t2, t3 = self.calc_balance_angle(h)
        print('Angle1: %3f' % t1)
        print('Angle2: %3f' % t2)
        print('Angle3: %3f' % t3)

    def setXVel(self, Ev):  # 注意：pitch方向和车方向相反，前倾为负
        # 对比和上次的位置，用count记录累加位移的量。
        # if self.panel.gps_v < Ev:  # 之前只能做到世界坐标系下x方向的位置平衡
        #     self.count = self.count + 1
        # elif self.panel.gps_x > Ev:
        #     self.count = self.count - 1
        # # print('count:%.3f' % self.count)
        #
        # if self.count > 100:  # 抗漂移的部分
        #     self.motors[4].setTorque(0.05 * self.factor1)
        #     self.motors[5].setTorque(0.05 * self.factor1)
        #     self.factor1 = self.factor1 + 0.1  # 使力矩递增的参数（因为在高速漂起来时，小恒力拉不回来。而大恒力在接近静止时会造成不稳定，故写成线性递增式力矩）
        #     self.count = self.count - 2  # 可以理解为纠正频率，减的数越小，纠正频率越高
        # elif self.count < -100:  # 抗漂移的部分
        #     self.motors[4].setTorque(-0.05 * self.factor2)
        #     self.motors[5].setTorque(-0.05 * self.factor2)
        #     self.factor2 = self.factor2 + 0.1  # 使力矩递增的参数（因为在高速漂起来时，小恒力拉不回来。而大恒力在接近静止时会造成不稳定，故写成线性递增式力矩）
        #     self.count = self.count + 2  # 可以理解为纠正频率，加的数越小，纠正频率越高
        #     # print('i2%.2f' % i_2)
        # else:
        # 参数过大后降参数，减小平衡后的晃动。可以理解为抗漂移给出的力矩波形，为周期性三角锯齿波（开始瞎编）
        # if self.factor1 >= 4:
        #     factor1 = 1
        # if self.factor2 >= 4:
        #     factor2 = 1

        # PID部分
        # if Ev > 0 and self.panel.bodyVel < Ev:
        #     v = self.panel.bodyVel*0.7
        #     self.Ev = self.panel.bodyVel_last*0.3+v
        # elif Ev < 0 and self.panel.bodyVel > Ev:
        #     v = self.panel.bodyVel * 0.7
        #     self.Ev = self.panel.bodyVel_last * 0.3 + v
        # if 0 < Ev < self.Ev:
        #     self.Ev = Ev
        # elif 0 > Ev > self.Ev:
        #     self.Ev = Ev
        if Ev == 0.0:
            angle = -0.04
        else:
            angle = -0.04

        # 直立
        pitch_err = angle - self.panel.pitch
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

        # wheel_err = self.Ev - self.panel.rightWheelVel*0.05  #
        # self.wheel_pid.feedback(wheel_err)
        # self.wheel_u = self.wheel_pid.get_u()

        # self.motors[4].setTorque(-self.blance_u + self.translation_Kp1 * self.translation_u)
        # self.motors[5].setTorque(-self.blance_u + self.translation_Kp1 * self.translation_u)
        self.motors[4].setTorque(
            -self.blance_u - self.omgz_u + self.translation_Kp1 * self.translation_u + self.wheel_Kp1 * self.wheel_u)
        self.motors[5].setTorque(
            -self.blance_u - self.omgz_u + self.translation_Kp1 * self.translation_u + self.wheel_Kp1 * self.wheel_u)

        print("b_u: %.5f" % self.blance_u)
        print("t_u: %.5f" % (self.translation_Kp1 * self.translation_u))
        # print("w_u: %.5f" % (self.wheel_Kp1 * self.wheel_u))
        # print("pitch_err: %.3f" % pitch_err)
        print("pitch: %.3f" % self.panel.pitch)
        print("omgz_u: %.3f" % (0.01 * self.omgz_u))
        # print("EV: %.3f" % (Ev))
        print("GPS_V: %.3f" % self.panel.gps_v)
        print("GPS_height: %.3f" % self.panel.gps_y)
        print("wheel_V: %.3f" % (self.panel.rightWheelVel * 0.05))
        print("body_V: %.3f" % self.panel.bodyVel)
        # print("omega_y: %.5f" % self.panel.omega_y)
        # print("omega_x: %.5f" % self.panel.omega_x)
        print("omega_z: %.5f" % self.panel.omega_z)
        print("期望速度： %.5f" % self.Ev)
        # print("与期望速度差： %.5f" % (Ev - self.panel.rightWheelVel * 0.05))
        print("预期倾角：%.5f" % angle)
        # print("Displacement: %.2f" % (self.panel.gps_dd))
        # print("rWheelVel: %.5f" % (self.panel.rightWheelVel))
        # print("rWheelVelSP: %.3f" % (self.panel.samplingPeriod))

    def jump(self, robot, panel):

        # samplingPeriod = self.motors[2].getTorqueFeedbackSamplingPeriod() / 1000
        # print('sam:%3f' % samplingPeriod)
        # self.motors[2].enableTorqueFeedback(samplingPeriod)
        self.motors[2].enableTorqueFeedback(1)
        self.motors[2].setPosition(0)
        self.motors[3].setPosition(0)
        tor2 = self.motors[2].getTorqueFeedback()
        velocity2 = self.motors[2].getVelocity()
        print('torque[2]:%3f' % tor2)
        print('Velocity[2]:%3f' % velocity2)

        self.motors[0].enableTorqueFeedback(1)
        self.motors[0].setPosition(0)
        self.motors[1].setPosition(0)
        tor0 = self.motors[2].getTorqueFeedback()
        velocity0 = self.motors[0].getVelocity()
        print('torque[0]:%3f' % tor0)
        print('Velocity[0]:%3f' % velocity0)

        last_v = 0
        count = 0
        while 1:
            # 在空中不采取行动
            TIME_STEP = int(robot.getBasicTimeStep())
            robot.step(TIME_STEP)
            panel.updateEncoder()
            panel.updateWheelVelocity()
            print('jumping rightWheelVel:', panel.rightWheelVel)
            # 轮子腾空后空转，转速先增后降，通过转速降来判断腾空后的落地过程（用GPS效果更好但是是流氓办法）
            if abs(last_v) > abs(panel.rightWheelVel):
                count += 1
                if count >= 3:
                    break  # 若轮子速度连续下降，即进入平衡车状态
            last_v = panel.rightWheelVel

            # if abs(panel.rightWheelVel) >= 5:
            #     while 1:
            #         TIME_STEP = int(robot.getBasicTimeStep())
            #         robot.step(TIME_STEP)
            #         panel.updateEncoder()
            #         panel.updateWheelVelocity()
            #         print('2::', panel.rightWheelVel)
            #         if abs(panel.rightWheelVel) < 5:
            #             break
            #     break

            # key = mKeyboard.getKey()  # 从键盘读取输入
            # if key == 66:
            #     break

        # self.motors[2].set_available_torque()
        # self.motors[3].set_available_torque()
        # self.motors[2].setPosition(float('+inf'))
        # self.motors[3].setPosition(float('+inf'))

        # self.motors[0].setTorque(-1)
        # self.motors[1].setTorque(-1)

        # self.motors[0].setPosition(float('+inf'))
        # self.motors[1].setPosition(float('+inf'))

# def setAVel(self,vel):
