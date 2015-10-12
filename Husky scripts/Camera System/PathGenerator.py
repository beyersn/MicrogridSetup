import numpy as np
from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt
from time import time
import sys

# Generate a set of paths


class PathGenerator(): #Create a regularly shaped path for a robot to drive at a fixed speed

    def __init__(self, path_type='circle', speed=.1):
        self.time = time()
        self.travel_distance = 0
        self.__path_type = path_type
        self.speed = speed
        self.x_array = []
        self.y_array = []
        self.distance_array = []
        self.total_distance = 1
        self.x_limit = 1
        self.y_limit = 1
        if path_type == 'circle':
            self.__generateCircle()
        elif path_type == 'infinity':
            self.__generateInfinity()
        elif path_type == 'square':
            self.__generateSquare()
        elif path_type == 'point':
            self.__generatePoint()
            self.speed = 0
        else:
            sys.exit('PathGeneration(): Wrong path type. Available options are "circle", "infinity", and "square".')
        # self.x = self.x_array[0]
        # self.y = self.y_array[0]

    def getPosition(self):
        time_now = time()
        self.travel_distance = (self.travel_distance + self.speed * (time_now - self.time)) % self.total_distance
        self.time = time()
        self.x = self.x_function(self.travel_distance)
        self.y = self.y_function(self.travel_distance)
        return self.x, self.y

    def __generateCircle(self):
        angle_steps = np.linspace(0, 2 * np.pi, num=101)
        self.x_array = self.x_limit * np.cos(angle_steps)
        self.y_array = self.x_limit * np.sin(angle_steps)
        self.distance_array = np.linspace(0, self.x_limit * 2 * np.pi, num=101)
        self.total_distance = self.x_limit * 2 * np.pi
        self.x_function = interp1d(self.distance_array, self.x_array, kind='quadratic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='quadratic')
        pass

    def __generateInfinity(self):
        angle_steps = np.linspace(0, 2 * np.pi, num=101)
        self.x_array = self.x_limit * np.cos(angle_steps)
        self.y_array = self.x_limit * np.sin(2 * angle_steps)
        x_steps = np.diff(self.x_array)
        y_steps = np.diff(self.y_array)
        hypot_steps = np.hypot(x_steps, y_steps)
        self.distance_array = np.insert(np.cumsum(hypot_steps), 0, 0)
        self.total_distance = self.distance_array[-1]
        self.x_function = interp1d(self.distance_array, self.x_array, kind='cubic')
        self.y_function = interp1d(self.distance_array, self.y_array, kind='cubic')

    def __generateSquare(self):
        self.x_function = interp1d(self.distance_array, self.x_array)
        self.y_function = interp1d(self.distance_array, self.y_array)
        pass

    def __generatePoint(self):
        x_reference = float(raw_input('Set X(meters):'))
        y_reference = float(raw_input('Set Y(meters):'))
        self.x = x_reference
        self.y = y_reference
        self.x_function = lambda x: x_reference
        self.y_function = lambda x: y_reference

    def printPos(self):
        print self.x
        print self.y
        print "--------------"