from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task
import umath as math

hub = PrimeHub()

def test_motor(port, duration, speed, repetitions, wait_duration = 1000):
    print("Speed:", speed)
    print("Wait Duration:", wait_duration)
    print("Motor:", port)
    motor = Motor(port)
    for i in range(repetitions):
        motor.reset_angle(0)
        
        print("Motor Limits:", motor.control.limits())
        print("Motor PID:", motor.control.pid())
        print("Max Voltage:", motor.settings())
        print("Hub Voltage:", hub.battery.voltage())
        print("Hub Current:", hub.battery.current())
        
        watch = StopWatch()
        motor.run(speed)
        wait(duration)
        print("Model State:", motor.model.state())
        motor.stop()
        print("Time:", watch.time())
        print("Motor Angle:", motor.angle())
        print("")
        wait(wait_duration)

if __name__ == "__main__":
    test_motor(Port.A, 3000, -800, 25, 3000)
