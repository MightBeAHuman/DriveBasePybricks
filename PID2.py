from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task

hub = PrimeHub()

class PIDController:
    def __init__(self, hub: PrimeHub, left_motor: Motor, right_motor: Motor, cp: float, ci: float, cd: float):
        self.hub = hub
        self.imu = self.hub.imu
        self.ml = left_motor
        self.mr = right_motor
        
        self.cp = cp
        self.ci = ci
        self.cd = cd

    async def gyroStraight(self, distance: int, speed: int):
        distance_driven = 0
        self.ml.reset_angle(0)
        self.mr.reset_angle(0)
        self.imu.reset_heading(0)

        change = 0
        integral = 0

        while distance_driven < distance:
            heading = self.imu.heading()
            change = heading - change
            integral += change
            data = (heading, integral, change)
            speed_left, speed_right = self.PID_calc_speed(speed, data)
            self.ml.run(speed_left)
            self.mr.run(speed_right)
        self.ml.brake()
        self.mr.brake()
    
    def PID_calc_speed(self, speed: int, data: tuple, target_value: int = 0):
        """
        Calculates the speed of the left and the right motor to approximate the target value to the p value.
        return: (speed_left, speed_right)
        """
        
        p, i, d = data
        p -= target_value

        val = self.cp * speed * p + self.ci * speed * i + self.cd * speed * d

        return speed + val, -speed

ma, me = Motor(Port.A), Motor(Port.E)

async def main():
    db = PIDController(hub, ma, me, 6, 0.0, 0.02)
    await db.gyroStraight(10*360, 1*360)

run_task(main())