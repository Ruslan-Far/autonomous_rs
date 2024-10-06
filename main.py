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

line_sensor = ColorSensor(Port.S1)

BLACK = 9
WHITE = 85
REFLECTION_THRESHOLD = (BLACK + WHITE) / 2
P = 1.2

STRAIGHT_SPEED = 50
TURN_RATE = 20
action_timer = StopWatch()

CROSSROAD_THRESHOLD = -37
WHITE_THRESHOLD = 30


def turn(direction, delay):
	robot.drive(0, direction * TURN_RATE)
	action_timer.reset()
	while action_timer.time() < delay:
		error = line_sensor.reflection() - REFLECTION_THRESHOLD
		if direction == -1:
			if error >= -5:
				return True
		else:
			if error <= -5:
				return True
		wait(10)
	return False


def search(is_crossroad):
	robot.stop()
	if is_crossroad:
		turn(-1, 6000)
	else:
		turn(1, 33000)
	robot.stop()


def run():
	white_count = 0

	while True:
		error = line_sensor.reflection() - REFLECTION_THRESHOLD
		if error < CROSSROAD_THRESHOLD:
			search(True)
			error = line_sensor.reflection() - REFLECTION_THRESHOLD
		if error > WHITE_THRESHOLD:
			if white_count == 10:
				search(False)
				white_count = 0
				error = line_sensor.reflection() - REFLECTION_THRESHOLD
			else:
				white_count += 1
		else:
			white_count = 0
		print(error)
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, turn_rate)
		wait(10)


run()
