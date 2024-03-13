""" Lab 4
Names:       Blake Silton and Cameron McClure-Coleman
Description: controller file for the robot that will be used to test our cruise control code on the other robot.
this robot will move at varying speeds to determine of the cruise control bot can adapt in the desired ways.
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

# Constants measured within the simulation
OUTER_ENCODER_PER_REV = 65.77672
INNER_ENCODER_PER_REV = 47.68404



class Vehicle:
    def __init__(self, kp=KP):

        # Most of this stuff is copied from the provided my_controller file.
        # The majority don't even really NEED to be instance variables,
        # but I've made them so for convenience, just in case.
        # Create the Robot instance.
        self.robot = Robot()
        # set the base velocity to midpoint of min and max to start. it will vary between min and max
        # delta_velocity is used to calculate the rate of change in the velocity functions
        # velocity_funciton is the function to use for calculation
        self.max_base_velocity = 40
        self.min_base_velocity = 10
        self.base_velocity = ((self.max_base_velocity + self.min_base_velocity)/2)
        self.delta_base_velocity = 1
        self.last_velocity_change = int(self.robot.getTime())
        self.velocity_function = "triangle"
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

    def update_velocity(self):
        match self.velocity_function:
            case "triangle":
                self.velocity_triangle_function()
            case _:
                self.velocity_triangle_function()

    def velocity_triangle_function(self):
        # if enough time has passed
        if self.robot.getTime() - self.last_velocity_change >= 5:
            self.last_velocity_change = self.robot.getTime()
            # do triangle logic
            if self.min_base_velocity <= self.delta_base_velocity >= self.max_base_velocity:
                self.delta_base_velocity *= -1
            self.base_velocity += self.delta_base_velocity

    def step(self):
        return self.robot.step(self.timestep)

    def p_control(self):
        """Modifies motor speeds according to the given error and proportional constant"""
        speed_adjustment = self.kp * self.calc_ground_error() #TODO: make speed adjustment change with base speed changes
        self.left_motor.setVelocity(self.base_velocity - speed_adjustment)
        self.right_motor.setVelocity(self.base_velocity + speed_adjustment)

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

    def default_behavior(self):
        """Default behavior provided.

        Goes straight forward and prints sensor vals to console. Basically unused.
        """
        self.left_motor.setVelocity(BASE_VELOCITY)  # set the left motor (radians/second)
        self.right_motor.setVelocity(BASE_VELOCITY)  # set the right motor (radians/second)
        print(self.left_motor.getVelocity())
        print(self.left_ground_sensor.getValue())
        print(self.middle_ground_sensor.getValue())
        print(self.right_ground_sensor.getValue())
        print(self.right_distance_sensor.getValue())
        new_encoder_values = self.get_encoder_values()
        print(new_encoder_values)
        print('-------------------------')

# Main loop:
# - perform simulation steps until Webots stops the controller
def main():
    """Main function."""
    vehicle = Vehicle()
    while vehicle.step() != -1:
        # Follow line
        vehicle.p_control()
        # update velocity
        vehicle.update_velocity()

if __name__ == '__main__':
    main()
