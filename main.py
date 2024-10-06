#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import _thread
import time

# ------------------------------------------------------- WHILE DOING HW4

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
ultrasonic_motor = Motor(Port.C)

WHEEL_DIAMETER = 54.5
AXLE_TRACK = 114.3

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

line_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

BLACK = 9
WHITE = 85
REFLECTION_THRESHOLD = (BLACK + WHITE) / 2
# P = 1.2
P = 0.175

STRAIGHT_SPEED = 50
TURN_RATE = 20
ULTRASONIC_TURN_RATE = 800

action_timer = StopWatch()

CROSSROAD_THRESHOLD = -20
WHITE_THRESHOLD = 30
# OBSTACLE_THRESHOLD = 133
OBSTACLE_THRESHOLD = 143

OBSTACLE_START = 200

ANGLE = 90

is_stop = False


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


def drive_to_wall():
	white_count = 0

	while True:
		if ultrasonic_sensor.distance() <= OBSTACLE_START:
			robot.stop()
			break
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
		print("drive_to_wall")
		print(error)
		print("end_drive_to_wall")
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, turn_rate)
		wait(10)


def rotate_ultrasonic_sensor():
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ANGLE)
	dist = ultrasonic_sensor.distance()
	print("dist")
	print(dist)
	print("end_dist")
	if dist <= 200:
		robot.stop()
		robot.turn(ANGLE / 2)
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 0)


def check_black_line():
	global is_stop

	print("{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}")
	# while not (line_sensor.reflection() - REFLECTION_THRESHOLD < CROSSROAD_THRESHOLD and action_timer.time() > 10000):
	while not (line_sensor.reflection() - REFLECTION_THRESHOLD < CROSSROAD_THRESHOLD and action_timer.time() > 100000):
		pass
	print("STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	is_stop = True
	robot.stop()
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 0)
	ev3.speaker.beep()
	return True
	# return False


# def rotate_ultrasonic_sensor():
# 	i = 0
# 	while i < ANGLE:
# 		i += 5
# 		ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, i)
# 		if check_black_line():
# 			return True
# 	dist = ultrasonic_sensor.distance()
# 	print("dist")
# 	print(dist)
# 	print("end_dist")
# 	if dist <= 200:
# 		robot.stop()
# 		robot.turn(ANGLE / 2)
# 	while i > 0:
# 		i -= 5
# 		ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, i)
# 		if check_black_line():
# 			return True
# 	return False


def run():
	global is_stop

	ev3.speaker.beep()
	drive_to_wall()
	robot.turn(ANGLE)
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -ANGLE)
	ultrasonic_motor.reset_angle(0)
	# action_timer.reset()
	while True:
		if is_stop:
			robot.stop()
			print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
			return
		error = ultrasonic_sensor.distance() - OBSTACLE_THRESHOLD
		if error > 800:
			error = 50
		elif error > 100:
			error = 100
		elif error < -100:
			error = -100
		print("error")
		print(error)
		print("end_error")
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, -turn_rate)
		rotate_ultrasonic_sensor()


# ultrasonic_motor.run_target(TURN_RATE, ANGLE / 2)
# ultrasonic_motor.run_target(TURN_RATE, 2 * ANGLE)
# ultrasonic_motor.run_target(TURN_RATE, -ANGLE / 2)
# ultrasonic_motor.run_target(TURN_RATE, -ANGLE)
# ultrasonic_motor.run_target(TURN_RATE, 15)

# run()

action_timer.reset()
_thread.start_new_thread(run, ())
_thread.start_new_thread(check_black_line, ())
# time.sleep(1000000000)
while not is_stop:
    pass
time.sleep(1)
print("Потоки завершены")

# while True:
# 	robot.drive(STRAIGHT_SPEED, 30)
# 	ultrasonic_motor.dc(20)
# 	# ultrasonic_motor.run_target(TURN_RATE, ANGLE)
# 	print("--------------------------------------------------------")
# 	wait(10)

# while True:
# 	print(ultrasonic_sensor.distance())
# 	wait(1000)

# while True:
# 	print(line_sensor.reflection() - REFLECTION_THRESHOLD)
# 	wait(1000)
