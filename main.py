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

LENGTH = 2000
WIDTH = 1000
ANGLE = -105
WHEEL_DIAMETER = 54.5
# AXLE_TRACK = 114.3
AXLE_TRACK = 105.75
STRAIGHT_SPEED = 100
TURN_RATE = 0.995

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)
action_timer = StopWatch()
main_action_timer = StopWatch()

def drive_side(side, angle):
	robot.straight(side)
	wait(100)
	robot.turn(angle)
	wait(100)


def drive_rectangle():
	i = 0

	while i < 2:
		drive_side(LENGTH, ANGLE)
		drive_side(WIDTH, ANGLE)
		i += 1


def drive_straight(main_delay):
	main_action_timer.reset()
	while main_action_timer.time() < main_delay:
		robot.reset()
		action_timer.reset()
		print("1")
		while action_timer.time() < 2000:
			if main_action_timer.time() >= main_delay:
				robot.stop()
				robot.reset()
				return
			# robot.drive(STRAIGHT_SPEED, 0.2) # good
			robot.drive(STRAIGHT_SPEED, 0.1) # good
		robot.reset()
		action_timer.reset()
		print("0")
		while action_timer.time() < 2000:
			if main_action_timer.time() >= main_delay:
				robot.stop()
				robot.reset()
				return
			robot.drive(STRAIGHT_SPEED, 1)
	robot.stop()
	robot.reset()
	

def drive_turn(action_delay):
	robot.reset()
	action_timer.reset()
	print("turn")
	while action_timer.time() < action_delay:
		# robot.drive(0, -50)
		robot.drive(0, -30)
	robot.stop()
	robot.reset()


def run_task2():
	i = 0

	# while i < 3:
	# 	drive_rectangle()
	# 	i += 1
	while i < 2:
		drive_straight(20000) # 2m
		wait(2000)
		# robot.turn(ANGLE)
		drive_turn(3400) # 90d
		wait(2000)
		drive_straight(10000) # 1m
		wait(2000)
		# robot.turn(ANGLE)
		drive_turn(3400) # 90d
		wait(2000)
		i += 1


# run_task2()
# robot.turn(ANGLE)
# drive_turn(2050)
# drive_turn(3400)


# -------------------------------- TASK 3

color_sensor = ColorSensor(Port.S1)

BLACK = 9
WHITE = 85
REFLECTION_THRESHOLD = (BLACK + WHITE) / 2
P = 1.2

STRAIGHT_SPEED = 50
TURN_RATE = 20
action_timer = StopWatch()
left_count = 0

WHITE_THRESHOLD = 30


def turn(direction, delay):
	robot.drive(0, direction * TURN_RATE)
	action_timer.reset()
	while action_timer.time() < delay:
		error = color_sensor.reflection() - REFLECTION_THRESHOLD
		if error <= -25: # чтобы робот смог выехать на левый край черной линии
			return True
	return False


def search():
	global left_count

	robot.stop()
	if left_count >= 1:
		is_found = turn(-1, 6000)
		if is_found:
			left_count += 1
			return is_found
		left_count = 0
		is_found = turn(1, 12000)
		if is_found:
			return is_found
	else:
		is_found = turn(1, 6000)
		if is_found:
			left_count = 0
			return is_found
		is_found = turn(-1, 12000)
		if is_found:
			left_count += 1
			return is_found
	return False


def run_task3():
	white_count = 0
	is_found = True

	while True:
		error = color_sensor.reflection() - REFLECTION_THRESHOLD
		if error > WHITE_THRESHOLD:
			if white_count == 10:
				is_found = search()
				robot.stop()
				if not is_found:
					return
				white_count = 0
				error = color_sensor.reflection() - REFLECTION_THRESHOLD
			else:
				white_count += 1
		else:
			white_count = 0
		print(error)
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, turn_rate)


run_task3()


# -------------------------------- TASK 4

ultrasonic_sensor = UltrasonicSensor(Port.S4)

START_OBSTACLE_THRESHOLD = 2000
OBSTACLE_THRESHOLD = 200
P = 0.5

def move_ultrasonic_sensor():
	action_timer.reset()
	while action_timer.time() < 120000:
		distance = ultrasonic_sensor.distance()
		if distance < OBSTACLE_THRESHOLD:
			print("in range of 200")
			while distance < OBSTACLE_THRESHOLD:                
				# robot.drive(-STRAIGHT_SPEED, 0)
				error = distance - OBSTACLE_THRESHOLD
				if error < -300:
					error = -300
				if error < 3 and error > -3:
					error = 0
				error *= 2 # чтобы назад ехал побыстрее
				robot.drive(P * error, 0)
				print(P * error)
				distance = ultrasonic_sensor.distance()
			robot.stop()
		elif distance > OBSTACLE_THRESHOLD:
			print("out range of 200")
			while distance > OBSTACLE_THRESHOLD:
				# robot.drive(STRAIGHT_SPEED, 0)
				error = distance - OBSTACLE_THRESHOLD
				if error > 300:
					error = 300
				if error < 3 and error > -3:
					error = 0
				robot.drive(P * error, 0)
				print(P * error)
				distance = ultrasonic_sensor.distance()
			robot.stop()
		else:
			print("200") 
			robot.stop()
	ev3.speaker.beep()
	print("Finish") 


def run_task4():
	while True:
		if ultrasonic_sensor.distance() > START_OBSTACLE_THRESHOLD:
			ev3.speaker.beep()
			move_ultrasonic_sensor()      
			break


# run_task4()
