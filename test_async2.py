from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
import umath as math

async def test1(ma):
    await test2(ma)

async def test2(ma):
    await wait(1000)
    await ma.run_angle(100, 360)
    print("Hi2")

async def test3(ml):
    print("Hello")
    await wait(500)
    print("Hi")
    await wait(2000)
    await ml.run_angle(400, 360)
    print("World")

if __name__ == "__main__":
    ml, mr = Motor(Port.B), Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
    ma = Motor(Port.A)
    hub = PrimeHub()


    run_task(multitask(test1(ma), test3(ml)))

