from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
import umath as math

class MovementAction:
    def __init__(self, movement: func, *args, **kwargs):
        self.movement = movement
        self.args = args
        self.kwargs = kwargs

    async def __call__(self):
        await self.movement(*self.args, **self.kwargs)
        await wait(100)

    async def execute(self):
        await self()


class ConditionalAction:
    def __init__(self, *actions, condition: func):
        self.condition = condition
        self.actions = actions

    async def __call__(self):
        while not self.condition():
            pass
        for action in self.actions:
            await action()

    async def execute(self):
        await self()

class ParallelAction:
    def __init__(self, *actions, race: bool = False):
        self.actions = actions
        self.race = race

    async def __call__(self):
        if self.race: await multitask(*[action() for action in self.actions], race=True)
        else: await multitask(*[action() for action in self.actions])
 

    async def execute(self):
        await self()

class Action:
    def __init__(self):
        self.actions = []

    def parallel(self, *actions, race: bool = False):
        a = ParallelAction(*actions, race=race)
        self.actions.append(a)
        return self

    def conditional(self, *actions, condition: func):
        a = ConditionalAction(*actions, condition=condition)
        self.actions.append(a)
        return self

    def move(self, movement: func, *args, **kwargs):
        a = MovementAction(func, *args, **kwargs)
        self.actions.append(a)
        return self
    
    async def __call__(self):
        for action in self.actions:
            await action()

    async def execute(self):
        self()

async def yaw(hub, ml, mr, deg, min_velocity: int = 300, max_velocity: int = 20):
    deg = deg % 360
    time_limit = 3000
    s = StopWatch()
    start = s.time()
    while True:
        current_yaw = (hub.imu.heading()) % 360
        if current_yaw < 0: current_yaw = 360 - current_yaw
        difference = deg - current_yaw


        if abs(difference) < 0.1:
            ml.stop()
            mr.stop()
            break

        if s.time() - start > time_limit:
            print("Timeout")
            break


        if abs(difference) > 180:
            difference = (360 - abs(difference)) * -1 * (difference/abs(difference))
        
        direction = 1 if difference >= 0 else -1

        max_velocity = 300
        min_velocity = 20

        # higher values = higher average speed
        speed_potency = 3

        velocity = round(min_velocity + (max_velocity - min_velocity) * (abs(difference) / 180) ** (1 / speed_potency))

        ml.run(-velocity * direction)
        mr.run(velocity * direction)

    ml.stop()
    mr.stop()
    print("Gyro-Sensor: " + str(hub.imu.heading()))

async def run_motor(ma):
    await ma.run_angle(700, 1260)
    await(100)

async def run_motor2(ml):
    await ml.run_angle(100, 720)
    await(100)

async def main(hub, ml, mr, ma):
    a = Action
    a.parallel(MovementAction(ml.run_angle, 100, 360), ConditionalAction(MovementAction(ma.run_angle, 700, 360), condition = lambda: hub.imu.heading() <= -30), race=True)
    await a.execute()
    await wait(2000)

    await multitask(MovementAction(ma.run_angle, 700, 360)(), MovementAction(ml.run_angle, 100, 720)(), race=False)
    await wait(2000)
    await multitask(ma.run_angle(700, 1260), yaw(hub, ml, mr, 90))


if __name__ == "__main__":
    hub = PrimeHub()
    ml = Motor(Port.B)
    mr = Motor(Port.F, positive_direction = Direction.COUNTERCLOCKWISE)
    ma = Motor(Port.A)
    run_task(main(hub, ml, mr, ma))
