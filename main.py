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

# ------------------------------------------------------- HW4

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
ultrasonic_motor = Motor(Port.C)

WHEEL_DIAMETER = 54.5
AXLE_TRACK = 114.3

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

BLACK = 7
WHITE = 23
REFLECTION_THRESHOLD = (BLACK + WHITE) / 2
P = 0.19

STRAIGHT_SPEED = 50
TURN_RATE = 20
ULTRASONIC_TURN_RATE = 800

action_timer = StopWatch()
black_line_timer = StopWatch()

CROSSROAD_THRESHOLD = -2
WHITE_THRESHOLD = 5
START_OBSTACLE_THRESHOLD = 200
OBSTACLE_THRESHOLD = 200
FORWARD_OBSTACLE_THRESHOLD = 300
CORNER_ERROR_THRESHOLD = 600
ERROR_THRESHOLD = 70

ANGLE = 100

is_stop = False


def turn(direction, delay):
	robot.drive(0, direction * TURN_RATE)
	action_timer.reset()
	while action_timer.time() < delay:
		error = color_sensor.reflection() - REFLECTION_THRESHOLD
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
		turn(-1, 6000)
	else:
		turn(1, 33000)
	robot.stop()


def drive_to_wall():
	white_count = 0

	while True:
		if ultrasonic_sensor.distance() <= START_OBSTACLE_THRESHOLD:
			robot.stop()
			break
		error = color_sensor.reflection() - REFLECTION_THRESHOLD
		if error < CROSSROAD_THRESHOLD:
			search(True)
			error = color_sensor.reflection() - REFLECTION_THRESHOLD
		if error > WHITE_THRESHOLD:
			if white_count == 10:
				search(False)
				white_count = 0
				error = color_sensor.reflection() - REFLECTION_THRESHOLD
			else:
				white_count += 1
		else:
			white_count = 0
		print("drive_to_wall_error")
		print(error)
		print("end_drive_to_wall_error")
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, turn_rate)


def rotate_ultrasonic_sensor():
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ANGLE)
	dist = ultrasonic_sensor.distance()
	if dist <= FORWARD_OBSTACLE_THRESHOLD:
		local_timer = StopWatch()
		local_timer.reset()
		while (local_timer.time() < 500):
			robot.drive(90, TURN_RATE * 3)
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 0)


def check_black_line():
	global is_stop

	black_line_timer.reset()
	while not (color_sensor.reflection() - REFLECTION_THRESHOLD < CROSSROAD_THRESHOLD and black_line_timer.time() > 50000):
		pass
	is_stop = True


def run():
	global is_stop

	ev3.speaker.beep()
	drive_to_wall()
	robot.turn(ANGLE - 20)
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -ANGLE)
	ultrasonic_motor.reset_angle(0)
	while True:
		if is_stop:
			robot.stop()
			return
		dist = ultrasonic_sensor.distance()
		error = dist - OBSTACLE_THRESHOLD
		if error > CORNER_ERROR_THRESHOLD:
			error = ERROR_THRESHOLD
		elif error > ERROR_THRESHOLD:
			error = ERROR_THRESHOLD
		elif error < -ERROR_THRESHOLD:
			error = -ERROR_THRESHOLD
		turn_rate = P * error
		robot.drive(STRAIGHT_SPEED, -turn_rate)
		rotate_ultrasonic_sensor()


# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 40) # по часовой стрелке
# ultrasonic_motor.run_target(TURN_RATE, 5) # по часовой стрелке
# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -10) # против часовой стрелки
# ultrasonic_motor.run_target(TURN_RATE, -20) # против часовой стрелки


_thread.start_new_thread(run, ())
_thread.start_new_thread(check_black_line, ())
while not is_stop:
    pass
time.sleep(1)
print("Потоки завершены")
ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ANGLE)
ev3.speaker.beep()
