from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task

hub = PrimeHub()

class PIDController:
    def __init__(self, hub: PrimeHub, left_motor: Motor, right_motor: Motor):
        self.hub = hub
        self.imu = self.hub.imu
        self.ml = left_motor
        self.mr = right_motor
        
        self.cp = -0.01
        self.bp = -6

        self.ci = 0.01
        self.bi = 4.0

        self.cd = 0.04
        self.bd = 4.0

    async def gyroStraight(self, distance: int, speed: int, debug: bool = False):
        s = StopWatch()
        distance_driven = 0
        self.ml.reset_angle()
        self.mr.reset_angle()
        self.imu.reset_heading(0)

        heading = self.imu.heading()
        prev_heading = self.imu.heading()
        integral = 0
        if debug: s.reset

        while s.time() < 10000:# distance_driven < distance:
            prev_heading = heading
            heading = self.imu.heading()
            change = heading - prev_heading
            integral += change

            p, i, d = self.calc_PID(speed)
            pid_value = (p * heading + i * integral + d * change)
            pid_value = max(-100, min(100, pid_value)) / 100

            print(f"PID: {pid_value}\nDIS: {distance_driven}\nGYR: {heading}\n")

            self.ml.run(speed * (1 + pid_value if pid_value < 0 else 1))
            self.mr.run(-speed * (1 - pid_value if pid_value > 0 else 1))
            distance_driven = self.ml.angle()
        self.ml.brake()
        self.mr.brake()
    
    def calc_PID(self, speed: int, target_value: int = 0):
        p = self.cp * speed + self.bp
        i = self.ci * speed + self.bi
        d = self.cd * speed + self.bd

        return p, i, d

ma = Motor(Port.F)
me = Motor(Port.B)

async def main():
    db = PIDController(hub, ma, me)
    await db.gyroStraight(10*360, 360, debug=True)

run_task(main())
