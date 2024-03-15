""" Lab 4
Names:       Blake Silton and Cameron McClure-Coleman
Description: Search and Destroy. Use Monte Carlo localization to figure out the simulated robot's
             position and take out the target.
"""
from controller import Robot, Motor
from random import random, choices
from math import sqrt, log, cos, pi, isnan
from statistics import mode, stdev

LEFT = 0
MIDDLE = 1
RIGHT = 2

LEFT_MOTOR = 0
RIGHT_MOTOR = 1

KP = 0.001  # Proportional control constant


class Vehicle:
    def __init__(self, kp=KP):
        # Most of this stuff is copied from the provided my_controller file.
        # The majority don't even really NEED to be instance variables,
        # but I've made them so for convenience, just in case.
        # Create the Robot instance.
        self.robot = Robot()
        # velocity
        self.target_velocity = 25
        self.current_velocity = 25

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())
        # enable the drive motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        # enable ground color sensors
        self.left_ground_sensor = self.robot.getDevice('gs0')
        self.left_ground_sensor.enable(self.timestep)
        self.middle_ground_sensor = self.robot.getDevice('gs1')
        self.middle_ground_sensor.enable(self.timestep)
        self.right_ground_sensor = self.robot.getDevice('gs2')
        self.right_ground_sensor.enable(self.timestep)
        # enable right distance sensor
        self.front_left_distance_sensor = self.robot.getDevice('ps7')  # IR sensor pointing to the right
        self.front_left_distance_sensor.enable(self.timestep)
        self.front_right_distance_sensor = self.robot.getDevice('ps0')
        self.front_right_distance_sensor.enable(self.timestep)
        # initialize encoders
        self.last_encoder_values = [None, None]
        self.encoders = []
        encoder_names = ['left wheel sensor', 'right wheel sensor']
        for i in range(2):
            self.encoders.append(self.robot.getDevice(encoder_names[i]))
            self.encoders[i].enable(self.timestep)
        # After this point is mostly new stuff.
        # Ensure encoders have time to initialize to non-NaN vals
        while any([isnan(e) for e in self.get_encoder_values()]):
            self.robot.step(self.timestep)
        self.kp = kp  # Proportional control constant

    def step(self):
        return self.robot.step(self.timestep)

    def p_control(self):
        """Modifies motor speeds according to the given error and proportional constant"""
        speed_adjustment = self.kp * self.calc_ground_error() # todo: make adjustment proportional to current speed
        self.left_motor.setVelocity(self.current_velocity - speed_adjustment)
        self.right_motor.setVelocity(self.current_velocity + speed_adjustment)

    def p_velocity_control(self, distance):
        return 0

    def i_velocity_control(self, distance):
        return 0

    def d_velocity_control(self, distance):
        return 0

    def pid_velocity_control(self, distance):
        too_far = 20
        # collect distances
        front_left_distance = self.front_left_distance_sensor.getValue()
        front_right_distance = self.front_right_distance_sensor.getValue()

        if front_left_distance <= too_far and front_right_distance <= too_far:
            return self.target_velocity
        elif front_left_distance > too_far:
            # something is in front of the robot. do pid logic to slow down in order to not hit it
            pid_correction = self.p_velocity_control(front_left_distance) + self.i_velocity_control(front_left_distance) + self.d_velocity_control(front_left_distance)
            return pid_correction
        elif front_right_distance > too_far:
            pid_correction = self.p_velocity_control(front_right_distance) + self.i_velocity_control(front_right_distance) + self.d_velocity_control(front_left_distance)
            return pid_correction


    def get_right_distance_reading(self) -> float:
        """Returns the current reading from the right distance sensor"""
        return self.right_distance_sensor.getValue()

    def get_ground_sensors(self) -> list:
        """Returns an array of the current ground sensor readings."""
        readings_arr = 3 * [None]
        readings_arr[LEFT] = self.left_ground_sensor.getValue()
        readings_arr[MIDDLE] = self.middle_ground_sensor.getValue()
        readings_arr[RIGHT] = self.right_ground_sensor.getValue()
        return readings_arr

    def calc_ground_error(self) -> float:
        """Calculates the error based on the ground sensor readings.

        - Very positive error -> left sensor reading is low (relative to other two)
        - Very negative error -> right sensor reading is low
        - Error close to 0 -> left and right differ from middle by similar amounts

        :return: total error between the side sensors as compared to the middle sensor
        """
        [left_sensor, middle_sensor, right_sensor] = self.get_ground_sensors()
        l_err = middle_sensor - left_sensor
        r_err = middle_sensor - right_sensor
        total_err = l_err - r_err
        return total_err

    def get_encoder_values(self, dryrun=False) -> list:
        """Returns the current encoder values.

        :param dryrun: If True, doesn't modify the last_encoder_values attribute, defaults to False
        :return: a len-2 list of the current left and right encoder values, plus Gaussian noise
        """
        [left, right] = [e.getValue() + add_noise(std_dev=0.02) for e in self.encoders]
        if not dryrun:
            self.last_encoder_values = [left, right]
        return [left, right]

    def default_behavior(self):
        """Default behavior provided.

        Goes straight forward and prints sensor vals to console. Basically unused.
        """
        self.left_motor.setVelocity(self.target_velocity)  # set the left motor (radians/second)
        self.right_motor.setVelocity(self.target_velocity)  # set the right motor (radians/second)
        print(self.left_motor.getVelocity())
        print(self.left_ground_sensor.getValue())
        print(self.middle_ground_sensor.getValue())
        print(self.right_ground_sensor.getValue())
        print(self.right_distance_sensor.getValue())
        new_encoder_values = self.get_encoder_values()
        print(new_encoder_values)
        print('-------------------------')

    def is_tower(self, position) -> bool:
        """Returns True if the position is expected to be adjacent to a tower. False otherwise."""
        assert 0 <= position < 360
        res = False
        for tower in self.tower_bounds:
            res = res or tower[0] <= position <= tower[1]
        return res

    def resample_particles(self):
        """Resamples the particles according to their current weights."""
        new_particles = choices(
            self.particles,
            weights=[p.weight for p in self.particles],
            k=self.num_particles - NUM_RANDOM_PARTICLES  # leave noise for random noise particles
        )
        for i in range(NUM_RANDOM_PARTICLES):
            new_particles.append(Particle())
        self.particles = new_particles
        # Now make a new best guess as to current position (the statistical mode of all positions)
        self.location = mode([p.position for p in self.particles])

    def turn_to_target(self):
        print('*krsh* take him down, over *krsh*')
        self.left_motor.setVelocity(BASE_VELOCITY)
        self.right_motor.setVelocity(-1 * BASE_VELOCITY)
        target_time = self.robot.getTime() + 2.3  # takes about 2.3 seconds to turn 90 degrees
        while self.robot.getTime() < target_time:
            self.step()
        self.right_motor.setVelocity(BASE_VELOCITY)


def add_noise(mean=0, std_dev=1.0):
    """Returns a value to use as Gaussian noise."""
    rand1 = random()
    rand2 = random()
    z = sqrt(-2 * log(rand1)) * cos(2 * pi * rand2)
    return mean + (std_dev * z)


def trapezoid(a, b, c, d, x, maximum, minimum):
    """a, b, c, d are distances from the sensor where the function changes slope
    min and max should always be 0 & 1 because this function is being used to set probabilities
       |
    max|       -------
       |      |        \\
       |     |           \\
    min|____|______________\\_______
            a  b      c    d
    """
    if x < a:
        res = 0.0001
    elif a < x <= b:
        res = (((x - a) / (b - a)) * (maximum - minimum)) + minimum
    elif b < x <= c:
        res = 1
    elif c < x <= d:
        res = (((x - c) / (d - c)) * (maximum - minimum)) + minimum
    else:  # d < x
        res = 0.0001
    return res


def trapezoid_free_space(dist):
    """Wrapper function for free space trapezoid so that numbers can be more easily tuned."""
    # a = 1.0
    # b = 2.0
    # c = 5.0
    # d = 10.0
    a = 0
    b = 48
    c = 80
    d = 150
    maximum = 1.0
    minimum = 0.0
    return trapezoid(a, b, c, d, dist, maximum, minimum)


def trapezoid_tower(dist):
    """Wrapper function for tower trapezoid so that numbers can be more easily tuned."""
    # a = 7.0
    # b = 12.0
    # c = 15.0
    # d = 17.0
    a = 80
    b = 120
    c = 230
    d = 300
    maximum = 1.0
    minimum = 0.0
    return trapezoid(a, b, c, d, dist, maximum, minimum)


# Main loop:
# - perform simulation steps until Webots stops the controller
def main():
    """Main function."""
    towers = TOWERS
    target = TARGET
    vehicle = Vehicle(towers=towers, target=target)
    target_tower_bounds = vehicle.tower_bounds[vehicle.target]
    print(f'Target tower_bounds: {target_tower_bounds}')
    while vehicle.step() != -1:
        # Follow line
        vehicle.p_control()
        std_dev = 100
        degrees_traveled = vehicle.update_position(dryrun=True)
        # Update every 5 degrees traveled
        if degrees_traveled > 5:
            degrees_traveled = vehicle.update_position()
            distance_reading = vehicle.get_right_distance_reading()
            vehicle.update_particles(degrees_traveled, distance_reading)
            vehicle.resample_particles()
            # print(vehicle.particles)
            print(f'{distance_reading=}')
            print(f'{vehicle.location=}')
            std_dev = stdev([p.position for p in vehicle.particles])
            print(f'{std_dev=}')
        # If current position is at target and vehicle is reasonably certain of its position, attack
        if (target_tower_bounds[0] < vehicle.location < target_tower_bounds[1] and
                std_dev < STD_DEV_THRESHOLD):
            print("BREAK BREAK BREAK")
            break
    # break out of while loop when sure of positions and at target tower position
    vehicle.turn_to_target()


if __name__ == '__main__':
    main()
