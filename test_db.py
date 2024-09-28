from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import math

hub = PrimeHub()

ml, mr = Motor(Port.A), Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)

diameter = 5.6
axle = 11.3

async def curve(radius, angle, speed=100):
    cl, cr = radius / (radius + axle / 2), (radius + axle) / (radius + axle / 2)
    sl, sr = speed * cl, speed * cr
    d = 2 * radius * math.pi * angle / 360
    dl, dr = cl*d*360/diameter, cr*d*360/diameter
    ml.run_angle(sl, dl)
    await mr.run_angle(sr, dr)

if __name__ == "__main__":
    await curve(20, 90)
