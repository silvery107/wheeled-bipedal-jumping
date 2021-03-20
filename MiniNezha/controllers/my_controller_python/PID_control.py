#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np


class PID_Controller:
    err_D = 0
    err_I = 0
    Kp = 0
    Kd = 0
    Ki = 0
    u = 0

    def __init__(self, Kp, Kd, Ki=0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def feedback(self, err):
        self.err_D = (err - self.err_D)
        self.err_I += err
        if self.err_I > 1100:
            self.err_I = 1100
        if self.err_I < -1100:
            self.err_I = -1100
        self.u = self.Kp * err + self.Kd * self.err_D + self.Ki * self.err_I
        self.err_D = err

    def get_u(self):
        return self.u
