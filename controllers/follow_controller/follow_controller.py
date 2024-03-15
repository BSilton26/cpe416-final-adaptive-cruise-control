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

VELOCITY_KP = 0.001  # Proportional control for cruise-control velocity pid
ANGULAR_KP = 0.01
VELOCITY_KI = 0.1 # integral gain constant for cruise-control velocity pid
VELOCITY_KD = 0.1 # derivative gain constant for cruise-control velocity pid
FOLLOW_DISTANCE = 65 # distance sensor reading that is the minimum comfortable follow distance, PID will target this

class Vehicle:
    def __init__(self, angular_kp=ANGULAR_KP,
                 velocity_kp = VELOCITY_KP,
                 velocity_ki=VELOCITY_KI,
                 velocity_kd=VELOCITY_KD,
                 follow_distance=FOLLOW_DISTANCE):
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
        # Proportional control constants
        self.angular_kp = angular_kp
        self.velocity_kp = velocity_kp
        self.velocity_ki = velocity_ki
        self.velocity_kd = velocity_kd
        self.past_errors = [] # queue to maintain the past n error values for integral calculation
        self.follow_distance = follow_distance

    def step(self):
        return self.robot.step(self.timestep)

    def p_angular_control(self):
        """Modifies motor speeds according to the given error and proportional constant"""
        speed_adjustment = self.angular_kp * self.calc_ground_error() # todo: make adjustment proportional to current speed
        self.left_motor.setVelocity(self.current_velocity - speed_adjustment)
        self.right_motor.setVelocity(self.current_velocity + speed_adjustment)

    def p_velocity_control(self, error) -> float:
        """Modifies the target speed according to proportional constant"""
        target_velocity_adjustment = self.velocity_kp * error
        return round(target_velocity_adjustment)

    def i_velocity_control(self) -> float:
        # sum the errors
        accumulated_error = sum(self.past_errors)
        return accumulated_error * self.velocity_ki

    def d_velocity_control(self) -> float:
        return 0.0 # no derivative term; I don't wanna.

    def update_error(self, distance):
        # calculate current error
        if distance <= self.follow_distance:
            current_error = distance - self.follow_distance
        else:
            current_error = 0
        # add the current error to the recent error queue
        if len(self.past_errors) >= 10:
            self.past_errors = self.past_errors[:10]
            self.past_errors.append(current_error)

    def pid_velocity_correction(self):
        too_far = self.follow_distance
        # collect distances
        front_left_distance = self.front_left_distance_sensor.getValue()
        front_right_distance = self.front_right_distance_sensor.getValue()
        closest_distance = max(front_left_distance, front_right_distance)
        self.update_error(closest_distance)

        if closest_distance <= too_far:
            self.current_velocity = self.target_velocity
        else:
            # something is in front of the robot. do pid logic to slow down in order to not hit it
            pid_correction = (self.p_velocity_control(closest_distance) +
                              self.i_velocity_control() +
                              self.d_velocity_control())
            self.current_velocity = pid_correction

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
        [left, right] = [e.getValue() for e in self.encoders]
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

# Main loop:
# - perform simulation steps until Webots stops the controller
def main():
    """Main function."""
    vehicle = Vehicle()
    while vehicle.step() != -1:
        # Follow line
        vehicle.pid_velocity_correction()
        vehicle.p_angular_control()
        # Update every 5 degrees traveled
        # If current position is at target and vehicle is reasonably certain of its position, attack
    # break out of while loop when sure of positions and at target tower position


if __name__ == '__main__':
    main()
