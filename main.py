#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

WHEEL_DIAMETER = 54.5
AXLE_TRACK = 114.3

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

color_sensor = ColorSensor(Port.S1)

REFLECTION_THRESHOLD = 52
P = 1

STRAIGHT_SPEED = 50
TURN_RATE = 20
action_timer = StopWatch()

CROSSROAD_THRESHOLD = -20
WHITE_THRESHOLD = 70


def turn(direction, delay, is_after_2):
	if is_after_2:
		robot.drive(STRAIGHT_SPEED, direction * TURN_RATE)
	else:
		robot.drive(0, direction * TURN_RATE)
	action_timer.reset()
	while action_timer.time() < delay:
		rgb = color_sensor.rgb()
		rgb_sum = rgb[0] + rgb[1] + rgb[2]
		error = rgb_sum - REFLECTION_THRESHOLD
		if direction == -1:
			if error >= 0: # 0 - это левый край черной линии
				return True
		else:
			if error <= 0: # 0 - это левый край черной линии
				return True
	return False


def search(is_crossroad):
	robot.stop()
	if is_crossroad:
		# turn(-1, 6000, False)
		turn(-1, 2000, False)
		# robot.stop()
		# turn(-1, 6000, True)
	else:
		turn(1, 33000, False)
	robot.stop()


def run():
	white_count = 0

	while True:
		rgb = color_sensor.rgb()
		rgb_sum = rgb[0] + rgb[1] + rgb[2]
		error = rgb_sum - REFLECTION_THRESHOLD
		if error < CROSSROAD_THRESHOLD:
			search(True)
			error = rgb_sum - REFLECTION_THRESHOLD
		if error > WHITE_THRESHOLD:
			if white_count == 10:
				search(False)
				white_count = 0
				error = rgb_sum - REFLECTION_THRESHOLD
			else:
				white_count += 1
		else:
			white_count = 0
		print(error)
		if error > 25:
			error = 25
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, turn_rate)
		# wait(100)


run()



# HSV
# floor: [252, 42, 60]

# RGB
# floor: [45, 40, 68]