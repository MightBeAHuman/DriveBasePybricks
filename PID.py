from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task
import umath as math


class PID:
    def __init__(self, ml, mr):
        self.imu = PrimeHub().imu
        self.ml = ml
        self.mr = mr

        self.Kp = 1
        self.Ki = 1
        self.Kd = 1

        self.error = 0
        self.previous = 0

    def calc_straight(self, avg_speed):
        heading = self.imu.heading()
        p = heading
        i = self.error + heading % 360
        d = heading - self.previous

        self.previous = heading
        self.error = i

        p *= self.Kp
        i *= self.Ki
        d *= self.Kd

        



        
