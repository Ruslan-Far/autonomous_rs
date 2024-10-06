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
ANGLE = -90
WHEEL_DIAMETER = 54.5
AXLE_TRACK = 114.3

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)


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


def run_task2():
	i = 0

	while i < 3:
		drive_rectangle()
		i += 1


# run_task2()
# robot.turn(angle)
# wait(100)
# robot.turn(angle)
# wait(100)
# robot.turn(angle)
# wait(100)
# robot.turn(angle)


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


def move_ultrasonic_sensor():
    action_timer.reset()
    while action_timer.time() < 120000:
        if ultrasonic_sensor.distance() < OBSTACLE_THRESHOLD:
            while ultrasonic_sensor.distance() < OBSTACLE_THRESHOLD:                
                robot.drive(-STRAIGHT_SPEED, 0)
            print("after 200") 
            robot.stop()
        elif ultrasonic_sensor.distance() > OBSTACLE_THRESHOLD:
            while ultrasonic_sensor.distance() > OBSTACLE_THRESHOLD:
                robot.drive(STRAIGHT_SPEED, 0)
            print("before 200") 
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


print(ultrasonic_sensor.distance()) 
run_task4()
