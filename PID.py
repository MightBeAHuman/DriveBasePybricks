from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
import umath as math


class PID:
    def __init__(self, ml, mr):
        self.imu = PrimeHub().imu
        self.ml = ml
        self.mr = mr

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
        return (avg_speed * (1 - correction_ratio), avg_speed * correction_ratio)

    async def straight(self, speed):
        self.running = True
        while self.running:
            vl, vr = self.calc_straight(speed)
            self.ml.run(vl)
            self.mr.run(vr)
            await wait(1)

    def stop(self):
        self.running = False

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))

async def main(pid):
    await wait(5000)
    pid.stop()

if __name__ == "__main__":
    ma, me = Motor(Port.B, positive_direction = Direction.COUNTERCLOCKWISE), Motor(Port.F)
    pid = PID(ma, me)
    run_task(multitask(pid.straight(2*800), main(pid), race=True))
