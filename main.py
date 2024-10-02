#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# -------------------------------- TASK 2

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

length = 2000
width = 1000
angle = -90
wheel_diameter = 54.5
axle_track = 114.3

robot = DriveBase(left_motor, right_motor, wheel_diameter=wheel_diameter, axle_track=axle_track)


def drive_side(side, angle):
	robot.straight(side)
	wait(100)
	robot.turn(angle)
	wait(100)


def drive_rectangle():
	global length
	global width

	i = 0

	while i < 2:
		drive_side(length, angle)
		drive_side(width, angle)
		i += 1


def run():
	i = 0

	while i < 3:
		drive_rectangle()
		i += 1


run()
# robot.turn(angle)
# wait(100)
# robot.turn(angle)
# wait(100)
# robot.turn(angle)
# wait(100)
# robot.turn(angle)


# ev3 = EV3Brick()
# MOTOR_A = Motor(Port.A, Direction.COUNTERCLOCKWISE)
# MOTOR_D = Motor(Port.D, Direction.COUNTERCLOCKWISE)
# color_sensor = ColorSensor(Port.S3)

# SPEED_A = 200

# angle = 90


# def run_motor(motor, speed, angle):
# 	motor.run_target(speed, angle)
# 	wait(100)


# ev3.speaker.beep()
# run_motor(MOTOR_A, SPEED_A, angle)
# run_motor(MOTOR_D, SPEED_A, angle)



# --------------------------------------------
# Initialize the motors connected to the drive wheels.
# left_motor = Motor(Port.A)
# right_motor = Motor(Port.D)

# Initialize the motor connected to the arms.
# arm_motor = Motor(Port.C)

# fall_timer = StopWatch()
# single_loop_timer = StopWatch()
# control_loop_timer = StopWatch()
# action_timer = StopWatch()

# wheel_diameter = 55
# axle_track = 120
# distance = 100

# DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
# DriveBase(left_motor, right_motor, wheel_diameter)
# drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=112)

# drive_base.straight(distance)

# run_motor(left_motor, SPEED_A, angle)
# run_motor(right_motor, SPEED_A, angle)










# from pybricks.hubs import EV3Brick
# from pybricks.ev3devices import Motor
# from pybricks.parameters import Port
# from pybricks.robotics import DriveBase

# # Initialize the EV3 Brick.
# ev3 = EV3Brick()

# # Initialize the motors.
# left_motor = Motor(Port.A)
# right_motor = Motor(Port.D)

# # Initialize the drive base.
# robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=120)

# # Go forward and backwards for one meter.
# robot.straight(100)
# ev3.speaker.beep()

# robot.straight(-100)
# ev3.speaker.beep()

# # Turn clockwise by 360 degrees and back again.
# robot.turn(360)
# ev3.speaker.beep()

# robot.turn(-360)
# ev3.speaker.beep()





# Initialize the motors.
# left_motor = Motor(Port.A)
# right_motor = Motor(Port.D)

# Initialize the color sensor.
# line_sensor = ColorSensor(Port.S1)

# rgb = line_sensor.rgb()
# print(rgb[0])
# print(rgb[1])
# print(rgb[2])

# Initialize the drive base.
# robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
# BLACK = 9
# WHITE = 85
# threshold = (BLACK + WHITE) / 2

# Set the drive speed at 100 millimeters per second.
# DRIVE_SPEED = 100

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
# PROPORTIONAL_GAIN = 1.2

# deviation = line_sensor.reflection() - threshold
# print(deviation)
# # Start following the line endlessly.
# while True:
#     # Calculate the deviation from the threshold.
	# deviation = line_sensor.reflection() - threshold
	# print(deviation)

#     # Calculate the turn rate.
	# turn_rate = PROPORTIONAL_GAIN * deviation

#     # Set the drive base speed and turn rate.
	# robot.drive(DRIVE_SPEED, turn_rate)

#     # You can wait for a short time or do other things in this loop.
	# wait(10)