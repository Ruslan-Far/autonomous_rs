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
import math

# ------------------------------------------------------- HW5

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
ultrasonic_motor = Motor(Port.C)

WHEEL_DIAMETER = 54.5
AXLE_TRACK = 114.3

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

# color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)

# BLACK = 7
# WHITE = 23
# REFLECTION_THRESHOLD = (BLACK + WHITE) / 2

P = 0.19

STRAIGHT_SPEED = 50
TURN_RATE = 20
ULTRASONIC_TURN_RATE = 800

# action_timer = StopWatch()

# CROSSROAD_THRESHOLD = -2
# WHITE_THRESHOLD = 5
# START_OBSTACLE_THRESHOLD = 200
OBSTACLE_THRESHOLD = 200
FORWARD_OBSTACLE_THRESHOLD = 200
# CORNER_ERROR_THRESHOLD = 600
ERROR_THRESHOLD = 70

ANGLE = 100

is_stop = False

GOAL_DISTANCE_THRESHOLD = 5

START_POSITION = (0, 0)
GOAL_POSITION = (225, 100)


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
	if ultrasonic_sensor.distance() <= FORWARD_OBSTACLE_THRESHOLD:
		local_timer = StopWatch()
		local_timer.reset()
		while (local_timer.time() < 500):
			robot.drive(1.8 * STRAIGHT_SPEED, 3 * TURN_RATE)
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 0)


def go_around_obstacle():
	error = ultrasonic_sensor.distance() - OBSTACLE_THRESHOLD
	if error > ERROR_THRESHOLD:
		error = ERROR_THRESHOLD
	elif error < -ERROR_THRESHOLD:
		error = -ERROR_THRESHOLD
	turn_rate = P * error
	robot.drive(STRAIGHT_SPEED, -turn_rate)
	rotate_ultrasonic_sensor()


def distance_to_goal(position):
    return math.sqrt((GOAL_POSITION[0] - position[0]) ** 2 + (GOAL_POSITION[1] - position[1]) ** 2)


def is_on_line(position):
	# x, y = position
	# x1, y1 = START_POSITION
	# x2, y2 = GOAL_POSITION
	y = K * position[0]
	print("abs(y - position[1])", abs(y - position[1]))
	return abs(y - position[1]) < 3
	# print("abs((y2 - y1) * (x - x1) - (x2 - x1) * (y - y1))", abs((y2 - y1) * (x - x1) - (x2 - x1) * (y - y1)))
	# return abs((y2 - y1) * (x - x1) - (x2 - x1) * (y - y1)) < 600


# def is_on_target():
# 	global is_stop

# 	while not is_stop and math.sqrt((Xg - x) ** 2 + (Yg - y) ** 2) > TARGET_DISTANCE_THRESHOLD: # если не стоп и не достигли целевой точки
# 		pass
# 	is_stop = True


# def run():
# 	global is_stop

# 	ev3.speaker.beep()
# 	drive_to_wall()
# 	robot.turn(ANGLE - 20)
# 	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -ANGLE)
# 	ultrasonic_motor.reset_angle(0)
# 	while True:
# 		if is_stop:
# 			robot.stop()
# 			return
# 		dist = ultrasonic_sensor.distance()
# 		error = dist - OBSTACLE_THRESHOLD
# 		if error > CORNER_ERROR_THRESHOLD:
# 			error = ERROR_THRESHOLD
# 		elif error > ERROR_THRESHOLD:
# 			error = ERROR_THRESHOLD
# 		elif error < -ERROR_THRESHOLD:
# 			error = -ERROR_THRESHOLD
# 		turn_rate = P * error
# 		robot.drive(STRAIGHT_SPEED, -turn_rate)
# 		rotate_ultrasonic_sensor()


def run():
	global K

	hit_position = None
	current_position = list(START_POSITION)
	prev_robot_distance = robot.distance()
	GOAL_ORIENTATION = math.acos(abs(START_POSITION[0] - GOAL_POSITION[0]) / distance_to_goal(START_POSITION)) * 180 / 3.14
	K = (GOAL_POSITION[1] - START_POSITION[1]) / (GOAL_POSITION[0] - START_POSITION[0])

	robot.turn(GOAL_ORIENTATION)
	current_robot_angle = robot.angle()
	current_orientation = abs(current_robot_angle) % 360
	if current_robot_angle < 0:
		current_orientation *= -1
	print("current_position[0]:", current_position[0])
	print("current_position[1]:", current_position[1])
	print("GOAL_ORIENTATION", GOAL_ORIENTATION)
	print("current_orientation:", current_orientation)
	print("math.cos(current_orientation)", math.cos(current_orientation * 3.14 / 180))
	print("math.sin(current_orientation)", math.sin(current_orientation * 3.14 / 180))
	# start of step 1
	while distance_to_goal(current_position) > GOAL_DISTANCE_THRESHOLD:  # Пока цель не достигнута (с запасом)
		# Движение по прямой линии
		while ultrasonic_sensor.distance() > FORWARD_OBSTACLE_THRESHOLD:  # Если путь свободен
			# print("distance_to_goal(current_position)", distance_to_goal(current_position))
			robot.drive(STRAIGHT_SPEED, 0)
			current_robot_distance = robot.distance()
			current_robot_angle = robot.angle()
			current_orientation = abs(current_robot_angle) % 360
			if current_robot_angle < 0:
				current_orientation *= -1
			current_position[0] += math.cos(current_orientation * 3.14 / 180) * (current_robot_distance - prev_robot_distance) / 10
			current_position[1] += math.sin(current_orientation * 3.14 / 180) * (current_robot_distance - prev_robot_distance) / 10
			prev_robot_distance = current_robot_distance
			print("1. current_position[0]:", current_position[0])
			print("1. current_position[1]:", current_position[1])
			print("1. current_orientation:", current_orientation)
			if distance_to_goal(current_position) <= GOAL_DISTANCE_THRESHOLD:
				print("1. цель достигнута")
				robot.stop()
				return

		print("обнаружено препятствие")
		robot.stop()
		hit_position = tuple(current_position)  # Запоминаем точку столкновения
		# end of step 1

		# start of step 2
		impossible_goal_timer = StopWatch()
		impossible_goal_timer.reset()
		print("начался обход препятствия против часовой стрелки")
		robot.turn(ANGLE - 20)
		current_robot_angle = robot.angle()
		current_orientation = abs(current_robot_angle) % 360
		if current_robot_angle < 0:
			current_orientation *= -1
		print("current_position[0]:", current_position[0])
		print("current_position[1]:", current_position[1])
		print("current_orientation:", current_orientation)
		# повернуть сонар к препятствию
		ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -ANGLE)
		ultrasonic_motor.reset_angle(0)
		while True:
			go_around_obstacle()

			current_robot_distance = robot.distance()
			current_robot_angle = robot.angle()
			current_orientation = abs(current_robot_angle) % 360
			if current_robot_angle < 0:
				current_orientation *= -1
			current_position[0] += math.cos(current_orientation * 3.14 / 180) * (current_robot_distance - prev_robot_distance) / 10
			current_position[1] += math.sin(current_orientation * 3.14 / 180) * (current_robot_distance - prev_robot_distance) / 10
			prev_robot_distance = current_robot_distance
			print("2. current_position[0]:", current_position[0])
			print("2. current_position[1]:", current_position[1])
			print("2. current_orientation:", current_orientation)

			# условие 3 из лекции: невозможность достижения цели
			if (impossible_goal_timer.time() > 5000) and (abs(current_position[0] - hit_position[0]) < 10 and abs(current_position[1] - hit_position[1]) < 10):
				print("невозможно достигнуть цели")
				robot.stop()
				return

			# Проверить, пересекаем ли прямую линию к цели
			if is_on_line(current_position) and distance_to_goal(current_position) < distance_to_goal(hit_position):
				print("обход препятствия завершен. Продолжаем движение к цели")
				robot.stop()
				print("robot.angle()", robot.angle())
				print("robot.distance()", robot.distance())
				# return
				# вернуть робота в исходную ориентацию
				robot.turn(GOAL_ORIENTATION - current_orientation)
				current_robot_angle = robot.angle()
				current_orientation = abs(current_robot_angle) % 360
				if current_robot_angle < 0:
					current_orientation *= -1
				print("current_position[0]:", current_position[0])
				print("current_position[1]:", current_position[1])
				print("current_orientation:", current_orientation)
				# вернуть сонар в исходную ориентацию
				ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ANGLE)
				ultrasonic_motor.reset_angle(0)
				break
		# end of step 2

	print("2. цель достигнута")
	robot.stop()


# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 40) # по часовой стрелке
# ultrasonic_motor.run_target(TURN_RATE, 5) # по часовой стрелке
# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -10) # против часовой стрелки
# ultrasonic_motor.run_target(TURN_RATE, -20) # против часовой стрелки


# _thread.start_new_thread(run, ())
# _thread.start_new_thread(is_on_target, ())
# while not is_stop:
    # pass
# time.sleep(1)
# print("Потоки завершены")
# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ANGLE)
# ev3.speaker.beep()
ev3.speaker.beep()
run()
ev3.speaker.beep()
