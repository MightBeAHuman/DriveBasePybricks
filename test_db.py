from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task
import umath as math

hub = PrimeHub()

async def arc(radius, angle, speed=200, clockwise: bool = True):
    if angle < 0: angle = -angle; speed=-speed
    diameter = 5.7
    axle = 11.3

    cl, cr = radius / (radius + axle / 2), (radius + axle) / (radius + axle / 2)
    if not clockwise: cl, cr = cr, cl
    sl, sr = speed * cl, speed * cr
    d = 2 * (radius + axle / 2) * math.pi * (angle / 360)
    dl, dr = cl*d/(diameter * math.pi)*360, cr*d/(diameter * math.pi)*360
    ml.run_angle(sl, (dl))
    await mr.run_angle(sr, (dr))

async def yaw():
    pass # + relative yaw
async def straight():
    pass # + gyro

async def move_motor():
    pass # Degrees and Rotations

async def start_db():
    pass # + gyro

async def stop_db():
    pass

async def elliptical_arc():
    pass

async def steer():
    pass # Spike Steering

async def color_recognition():
    pass

async def settings():
    # default_speed, acceleration, deceleration, PID, use_gyro

async def main():
    await curve(15, 180)
if __name__ == "__main__":
    mr, ml = Motor(Port.A), Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
    run_task(main())
