from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

wheel_diameter = 56 / 10
distance = 13 # in cm

pi = 3.14159265359

target_rotations = distance / (wheel_diameter * pi)

hub = PrimeHub()

pl, pr = Port.A, Port.E

# Motor ports: B (left), F (right)
motor_left, motor_right = Motor(pl), Motor(pr, positive_direction = Direction.COUNTERCLOCKWISE)

start_angle_left = motor_left.angle()
start_angle_right = motor_right.angle()

motor_left.run(311)
motor_right.run(311)

while True:
    rotations = (motor_left.angle() - start_angle_left) / 360
    if rotations >= target_rotations:
        motor_left.hold()
        motor_right.hold()
        break



