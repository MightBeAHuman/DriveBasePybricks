from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

pl, pr = Port.A, Port.E

# Motor ports: B (left), F (right)
motor_left, motor_right = Motor(pl), Motor(pr, positive_direction = Direction.COUNTERCLOCKWISE)
db = DriveBase(motor_left, motor_right, 55, 113)

db.straight(10*360)
