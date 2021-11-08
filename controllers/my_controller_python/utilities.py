#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import math

def maximum(a,b):
    if a>b:
        return a
    else:
        return b

def square_penalize(fx):
    return maximum(0,fx)**2