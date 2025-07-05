#!/usr/bin/python3
# coding=UTF-8

import time 
import random
import numpy as np
from pathlib import Path


localPath = Path(__file__).parent


class PID_Position:
    def __init__(self, Kp:float, Ki:float, Kd:float,Debug:bool = False):
        self.Kp:float = Kp
        self.Ki:float = Ki
        self.Kd:float = Kd
        self.Debug:bool = Debug
        self.clear()
        
    def clear(self):
        self.error:float = 0.0
        self.last_error:float = 0.0
        self.integral:float = 0.0
        self.derivative:float = 0.0
        self.output:float = 0.0

    def update(self, target_position, feedback_value):
        self.error = target_position - feedback_value
        if self.Debug:
            print(f"error: {self.error}")
        self.integral += self.error
        self.derivative = self.error - self.last_error
        self.last_error = self.error
        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        return self.output





if __name__ == '__main__':
    ...








