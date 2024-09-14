from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.tools import wait, StopWatch, run_task

hub = PrimeHub()

class DriveBase:
    # Init
    def __init__(self, hub: PrimeHub, left_motor: Motor, right_motor: Motor):
        
        # ----      HUB     ----- #

        self.hub = hub
        self.imu = self.hub.imu

        # -----    MOTORS   ----- #

        self.ml = left_motor
        self.mr = right_motor

        # -----     VARS    ----- #

        self.is_moving = False

        # ----- SET VALUES  ----- #

        self.imu.reset_heading(0)

        # -----    CONFIG   ----- #
        
        # CONFIG - PID
        self.cp = -0.01
        self.ci = 0.01
        self.cd = 0.04

        self.bp = -6
        self.bi = 4.0
        self.bd = 4.0

        return


    # ----- PUBLIC METHODS  ----- #
    #  ----     Movement    ----  #

    async def async_straight(self, distance: int, speed: int, use_gyro: bool = False):
        if use_gyro: 
            # self.is_moving = True
            self.ml.run(speed)
            self.mr.run(-speed)
            self.__straight_gyro(distance, speed)
        else:
            self.is_moving = True
            self.ml.run_angle(speed, distance)
            self.mr.run_angle(speed, distance)
            self.is_moving = False

    async def straight(self, distance: int, speed: int, use_gyro: bool = False):
        if use_gyro: 
            self.is_moving = True
            await self.__straight_gyro(distance, speed)
            self.is_moving = False
        else:
            self.is_moving = True
            self.ml.run_angle(speed, distance)
            await self.mr.run_angle(-speed, distance)
            self.is_moving = True

    async def distance(self, distance: int):
        while ((self.ml.angle() - self.mr.angle()) // 2) < distance: print((self.ml.angle() - self.mr.angle()) // 2)

    async def movement_done(self):
        while self.is_moving: pass
    
    async def run_motor(self, action: Motor, distance: int):
        await action.run_angle(360, distance)


    # ----- PRIVATE METHODS ----- #
    #  ----     Movement    ----  #

    async def __straight_gyro(self, distance: int, speed: int):
        distance_driven = 0
        self.ml.reset_angle()
        self.mr.reset_angle()
        self.imu.reset_heading(0)

        heading = self.imu.heading()
        prev_heading = self.imu.heading()
        integral = 0

        while True:
            if distance_driven >= distance: 
                self.ml.brake()
                self.mr.brake()
                self.is_moving = False

            prev_heading = heading
            heading = self.imu.heading()
            change = heading - prev_heading
            integral += change

            p, i, d = await self.__calc_PID(speed)
            pid_value = (p * heading + i * integral + d * change)
            pid_value = max(-100, min(100, pid_value)) / 100

            self.ml.run(speed * (1 + pid_value if pid_value < 0 else 1))
            self.mr.run(-speed * (1 - pid_value if pid_value > 0 else 1))
            distance_driven = self.ml.angle()

    #  ----   Calculations  ----  #

    async def __calc_PID(self, speed: int, target_value: int = 0):
        p = self.cp * speed + self.bp
        i = self.ci * speed + self.bi
        d = self.cd * speed + self.bd

        return p, i, d

# TEST
async def Fahrt1(d: DB):
    await d.straight(10*360, 360)
    # await d.distance(3*360)
    # await d.run_motor(m, 360)
    # await d.movement_done()
    # await d.run_motor(m, 360)

if __name__ == "__main__":
    hub = PrimeHub()
    db = DriveBase(hub, Motor(Port.A), Motor(Port.E))
    m = Motor(Port.B)
    run_task(Fahrt1(db))