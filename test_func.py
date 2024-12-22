from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task, multitask
import umath as math

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
async def main(hub, ml, mr):
    await yaw(hub, ml, mr, 90)

async def motor(ma):
    await ma.run_angle(900, 360)

if __name__ == "__main__":
    hub = PrimeHub()
    ml = Motor(Port.B)
    mr = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
    ma = Motor(Port.A)

    run_task(multitask(main(hub, ml, mr), motor(ma)))
