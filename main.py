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

action_timer = StopWatch()

P = 0.9

STRAIGHT_SPEED_BASE = 50 * 2.3
STRAIGHT_SPEED_CROSSROAD = 0
STRAIGHT_SPEED_WHITE_YELLOW = 0

TURN_RATE_CROSSROAD = 25
TURN_RATE_WHITE_YELLOW = 80

RGB_SUM_THRESHOLD = 52
CROSSROAD_THRESHOLD = -15
WHITE_YELLOW_THRESHOLD = 70
WHITE_YELLOW_COUNT_THRESHOLD = 50
UPPER_ERROR_THRESHOLD = 25 # ограничивает положительную ошибку при выезде с линии (для адекватной скорости поворота)


def turn(direction, delay):
	if delay == 2000: # is_crossroad
		robot.drive(STRAIGHT_SPEED_CROSSROAD, direction * TURN_RATE_CROSSROAD)
	else: # is_white_yellow
		robot.drive(STRAIGHT_SPEED_WHITE_YELLOW, direction * TURN_RATE_WHITE_YELLOW)
	action_timer.reset()
	while action_timer.time() < delay:
		rgb = color_sensor.rgb()
		rgb_sum = rgb[0] + rgb[1] + rgb[2]
		error = rgb_sum - RGB_SUM_THRESHOLD
		if direction == -1:
			if error >= 10: # 10 - это небольшое расстояние от левого края черной линии (подъезжаем справа)
				return True
		else:
			if error <= 0: # 0 - это левый край черной линии (подъезжаем слева)
				return True
	return False


def search(is_crossroad):
	robot.stop()
	if is_crossroad:
		print("is_crossroad")
		turn(-1, 2000)
	else:
		print("is_white_yellow")
		turn(1, 33000)
	robot.stop()


def run():
	white_yellow_count = 0

	while True:
		rgb = color_sensor.rgb()
		rgb_sum = rgb[0] + rgb[1] + rgb[2]
		error = rgb_sum - RGB_SUM_THRESHOLD
		if error < CROSSROAD_THRESHOLD:
			search(True)
			error = rgb_sum - RGB_SUM_THRESHOLD
		if error > WHITE_YELLOW_THRESHOLD:
			if white_yellow_count == WHITE_YELLOW_COUNT_THRESHOLD:
				search(False)
				white_yellow_count = 0
				error = rgb_sum - RGB_SUM_THRESHOLD
			else:
				white_yellow_count += 1
		else:
			white_yellow_count = 0
		print(error)
		if error > UPPER_ERROR_THRESHOLD:
			error = UPPER_ERROR_THRESHOLD
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED_BASE, turn_rate)


run()
