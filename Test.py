from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task
from DriveBase import DriveBase as DB

hub = PrimeHub()

db = DB(hub, Motor(Port.A), Motor(Port.E))
m = Motor(Port.B)

async def Fahrt1(d: DB):
    await d.async_straight(5*360, 360)
    await d.distance(3*360)
    await d.run_motor(m, 360)
    await d.movement_done()
    await d.run_motor(m, 360)

run_task(Fahrt1(db))