#!usr/bin/env python

from time import time


class PID(object):

    """docstring for ClassName"""

    def __init__(self, kp, ki, kd): #initialize with controller gains
        self.time = time()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.error_previous = 0

    def calculate(self, error):
        time_now = time()
        elapsed_time = time_now - self.time
        derivative_error = (error - self.error_previous) / elapsed_time #numeric derivative of error
        control_input = self.kp * error + self.ki * self.integral_error +\
            self.kd * derivative_error #determine PID control
        self.integral_error = self.integral_error + error * elapsed_time #numeric integral of error
        self.time = time_now
        self.error_previous = error

        return control_input

    def __str__(self):
        return str(self.kp)+' '+str(self.ki)+' '+str(self.kd) #returns a string with the controller values