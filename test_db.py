from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorDistanceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, run_task
import umath as math

hub = PrimeHub()

class DriveBase:
    def __init__(self, motor_left: Motor, motor_right: Motor, wheel_diameter: int, axle_track: int, negative_direction: bool = False, action_motors: dict = None, sensors: dict = None):
        """
        An interface used to abstract motor controlling into easy to use functions. Supports gyro stabilization.
        :param motor_left: Left motor
        :param motor_right: Right motor
        :param wheel_diameter: Average wheel diameter
        :param axle_track: Horizontal distance between the middle of both wheels
        :param negative_direction: Whether the main direction is negative
        :param action_motors: A dictionary containing names as keys and motors as values. These motors will be stored as class vars by their name.
        :param sensors: A dictionary containing names as keys and sensors as values. These sensors will be stored as class vars by their name.
        """


        self.ml = motor_left
        self.mr = motor_right
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.negative_direction = negative_directionA
        self.action_motors = action_motors if type(action_motors) == dict else {}
        self.sensors = sensors if type(sensors) == dict else {}

        for key, value in self.action_motors: setattr(self, key, value)
        for key, value in self.sensors: setattr(self, key, value)


    async def arc(radius, angle, speed=200, clockwise: bool = True):
        if angle < 0: angle = -angle; speed=-speed
        diameter = 5.7
        axle = 11.3
    
        cl, cr = radius / (radius + axle / 2), (radius + axle) / (radius + axle / 2)
        if not clockwise: cl, cr = cr, cl
        sl, sr = speed * cl, speed * cr
        d = 2 * (radius + axle / 2) * math.pi * (angle / 360)
        dl, dr = cl*d/(diameter * math.pi)*360, cr*d/(diameter * math.pi)*360
        ml.run_angle(sl, (dl))
        await mr.run_angle(sr, (dr))

    async def yaw(deg, min_velocity: int = 300, max_velocity: int = 20):
        deg = deg % 360
        time_limit = 3000
        start = time.ticks_ms()
        while True:
            current_yaw = (hub.motion_sensor.tilt_angles()[0]/10) % 360
            difference = deg - current_yaw
    
    
            if abs(difference) < 0.1:
                motor_pair.stop(motor_pair.PAIR_1)
                break
    
            if time.ticks_ms() - start > time_limit:
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
    
    
            motor_pair.move_tank(motor_pair.PAIR_1, velocity * direction, velocity * direction * -1)
        motor_pair.stop(motor_pair.PAIR_1)
        print("Gyro-Sensor: " + str(hub.motion_sensor.tilt_angles()[0]/10))

    async def straight():
        pass # + gyro
    
    async def move_motor():
        pass # Degrees and Rotations
    
    async def start_db():
        pass # + gyro
    
    async def stop_db():
        pass
    
    async def elliptical_arc():
        pass
    
    async def steer():
        pass # Spike Steering
    
    async def color_recognition():
        pass
    
    async def settings():
        # default_speed, acceleration, deceleration, PID, use_gyro
        pass
        
    def action_arc(drive_base, direction: int, action_motor, action_distance_per_rotation: int,  radius: int, angle: int, velocity: int, starting_angle: int = 0):
    
        """
        Moves any forklift like structure in an arc like motion
        
        :param drive_base: An drive base object using Spike Prime Drivebase (distances in mm)
        :param direction: 1 for forward and -1 for backward (regarding the drive base)
        :param action_motor: The action motor used for powering the forklift structure
        :param action_distance_per_rotation: The distance the forklift moves per rotation of action motor (negative values for negative rotation)
        :param radius: The radius of the arc in mm
        :param angle: The final angle of the arc motion
        :param velocity: Degrees of the arc motion per second
        :param starting_angle: The angle the arc motion starts at
        """
        
        # Calculates the time it takes for the whole arc to retrieve an bijective relation between time and angle
        # Converts all the angles to radiants for compatibility with cos and sin
        t_max = angle * 1000 / velocity
        angle = 2 * pi * ((angle - starting_angle) / 360)
        starting_angle = 2 * pi * (starting_angle / 360)
    
        # Using stopwatch for time measurement to calculate delta x and y
        s = StopWatch()
        x_prev = 0.0
        y_prev = 0.0
        t_prev = 0.0
        while (t:=s.time()) < t_max:
            # Calculate current expected angle phi using bijection between time and angle
            phi = angle * (t / t_max)
            
            # Calculate expected x and y for that angle to calculate delta x and y afterwards
            x, y = radius * cos(starting_angle + phi), radius * sin(starting_angle + phi)
            dt = (t - t_prev) / 1000 + (0.00001 if t - t_prev == 0 else 0)
            dx = (x - x_prev) / dt
            dy = (y - y_prev) / dt
            
            # Calculate speed using dx and dy and start all motors with their respective speed
            vx = dx * direction
            vy = dy * (360 / action_distance_per_rotation)
            action_motor.run(vy)
            drive_base.drive(vx, 0)
    
            # Store all values for next delta calculation
            t_prev = t
            x_prev = x
            y_prev = y
            wait(1)
    
        drive_base.stop()
        action_motor.stop()

async def main():
    await curve(15, 180)
if __name__ == "__main__":
    mr, ml = Motor(Port.A), Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
    run_task(main())
