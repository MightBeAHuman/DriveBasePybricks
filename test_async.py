from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
import umath as math

async def main1(motor):
    await motor.run_angle(100, 3*360)

def main3(motor, backwards):
    run_task(main2(motor, backwards))

def main4(motor):
    main3(motor)
    main3(motor, True)
    main3(motor)

async def main2(motor, backwards):
    await motor.run_angle((-1 if backwards else 1) * 100, 360)

if __name__ == "__main__":
    ma = Motor(Port.A)
    me = Motor(Port.E)
    run_task(multitask(main1(ma), main2(me, True)))
