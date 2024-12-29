from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
import umath as math


class PID:
    def __init__(self, ml, mr, wheel_diameter: int = None):
        self.imu = PrimeHub().imu
        self.ml = ml
        self.mr = mr
        self.wheel_diameter = abs(wheel_diameter) if wheel_diameter is not None else 0

        self.Kp = 0.15
        self.Tn = 1500
        self.Tv = 250
        self.tuning_speed = 800
        self.speed_multiplier = 1

        self.error = 0
        self.previous = 0

        self.running = False

    def calc_straight(self, avg_speed, debug: Bool = False):
        heading = self.imu.heading()
        p = heading
        i = self.error + heading
        d = heading - self.previous

        self.previous = heading
        self.error = i

        p *= self.Kp
        i *= self.Kp / self.Tn
        d *= self.Tv * self.Kp
        if debug: 
            print("P:" , p)
            print("Gyro:" , heading)

        correction_ratio = self.sigmoid((p + i + d) * self.speed_multiplier * (self.tuning_speed / avg_speed )) 
        return (2*avg_speed * (1 - correction_ratio), 2*avg_speed * correction_ratio)

    async def async_run_straight(self, speed, debug: bool = False):
        c = 1
        self.running = True
        while self.running:
            if debug: c += 1
            vl, vr = self.calc_straight(speed, c % 100 == 0)
            self.ml.run(vl)
            self.mr.run(vr)
            await wait(1)

    def run_straight(self, speed):
        self.running = True
        while self.running:
            vl, vr = self.calc_straight(speed)
            self.ml.run(vl)
            self.mr.run(vr)

    def straight(self, speed, distance):
        pass


    def stop(self):
        self.running = False

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

async def main(pid):
    await wait(5000)
    pid.stop()

if __name__ == "__main__":
    ma, me = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE), Motor(Port.F)
    ma.run(800)
    me.run(800)
    wait(5000)
    ma.stop()
    me.stop()
    wait(5000)
    pid = PID(ma, me)
    print(pid.sigmoid(0))
    #pid.run_straight(800)
    run_task(multitask(pid.async_run_straight(1*800, True), main(pid), race=True))
