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

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
ultrasonic_motor = Motor(Port.C)

WHEEL_DIAMETER = 56
AXLE_TRACK = 123

robot = DriveBase(left_motor, right_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

ultrasonic_sensor = UltrasonicSensor(Port.S1)
gyro_sensor = GyroSensor(Port.S2)

P = 0.19

STRAIGHT_SPEED = 80
TURN_RATE = 20
ULTRASONIC_TURN_RATE = 800

END_ANGLE = 90
ULTRASONIC_END_ANGLE = 100

OBSTACLE_THRESHOLD = 300
FORWARD_OBSTACLE_THRESHOLD = 150
ERROR_THRESHOLD = 70

GOAL_DISTANCE_THRESHOLD = 15
ON_LINE_THRESHOLD = 5

START_POSITION = (0, 0)
GOAL_POSITION = (300, 180) # x не должен быть равен 0!!!


def go_back_ultrasonic_sensor():
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ULTRASONIC_END_ANGLE)
	ultrasonic_motor.reset_angle(0)


def rotate_ultrasonic_sensor():
	ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, ULTRASONIC_END_ANGLE)
	if ultrasonic_sensor.distance() <= FORWARD_OBSTACLE_THRESHOLD:
		robot.stop()
		robot.turn(END_ANGLE)
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


def calc_distance_to_goal(position):
    return math.sqrt((position[0] - GOAL_POSITION[0]) ** 2 + (position[1] - GOAL_POSITION[1]) ** 2)


def is_on_line(position):
	if abs(GOAL_POSITION[0]) >= abs(GOAL_POSITION[1]):
		x = position[0]
		y = position[1]
	else:
		x = -1 * position[1]
		y = position[0]
	y_on_line = K * x
	print("abs(y - y_on_line)", abs(y - y_on_line))
	return abs(y - y_on_line) < ON_LINE_THRESHOLD


def get_correct_orientation():
	current_robot_angle = -1 * gyro_sensor.angle()
	current_orientation = abs(current_robot_angle) % 360
	if current_robot_angle < 0:
		current_orientation *= -1
	return current_orientation


def process_position(position, orientation, robot_distance, prev_robot_distance):
	position[0] += math.cos(orientation * math.pi / 180) * (robot_distance - prev_robot_distance) / 10
	position[1] += math.sin(orientation * math.pi / 180) * (robot_distance - prev_robot_distance) / 10


def run():
	global K

	hit_position = None
	current_position = list(START_POSITION)
	prev_robot_distance = robot.distance()
	# GOAL_POSITION[0] не должен быть равен START_POSITION[0]
	GOAL_ORIENTATION = math.atan2(GOAL_POSITION[1] - START_POSITION[1], GOAL_POSITION[0] - START_POSITION[0]) * 180 / math.pi # degrees
	k = (GOAL_POSITION[1] - START_POSITION[1]) / (GOAL_POSITION[0] - START_POSITION[0])
	if abs(GOAL_POSITION[0]) >= abs(GOAL_POSITION[1]):
		K = k
	else:
		K = -1 / k

	robot.turn(-1 * GOAL_ORIENTATION)
	current_orientation = get_correct_orientation()
	print("current_position[0]:", current_position[0])
	print("current_position[1]:", current_position[1])
	print("GOAL_ORIENTATION", GOAL_ORIENTATION)
	print("current_orientation:", current_orientation)
	print("math.cos(current_orientation)", math.cos(current_orientation * math.pi / 180))
	print("math.sin(current_orientation)", math.sin(current_orientation * math.pi / 180))
	# start of step 1
	while calc_distance_to_goal(current_position) > GOAL_DISTANCE_THRESHOLD:  # пока цель не достигнута
		# движение по прямой линии
		while ultrasonic_sensor.distance() > FORWARD_OBSTACLE_THRESHOLD:  # если путь свободен
			print("calc_distance_to_goal(current_position)", calc_distance_to_goal(current_position))
			robot.drive(STRAIGHT_SPEED, 0)
			current_orientation = get_correct_orientation()
			current_robot_distance = robot.distance()
			process_position(current_position, current_orientation, current_robot_distance, prev_robot_distance)
			prev_robot_distance = current_robot_distance
			print("1. current_position[0]:", current_position[0])
			print("1. current_position[1]:", current_position[1])
			print("1. current_orientation:", current_orientation)
			if calc_distance_to_goal(current_position) <= GOAL_DISTANCE_THRESHOLD:
				print("1. цель достигнута")
				robot.stop()
				return

		print("обнаружено препятствие")
		robot.stop()
		hit_position = tuple(current_position)  # Запоминаем точку столкновения
		# end of step 1

		# start of step 2
		departure_timer = StopWatch()
		departure_timer.reset()
		print("начался обход препятствия против часовой стрелки")
		robot.turn(END_ANGLE)
		current_orientation = get_correct_orientation()
		print("current_position[0]:", current_position[0])
		print("current_position[1]:", current_position[1])
		print("current_orientation:", current_orientation)
		# повернуть сонар к препятствию
		ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -ULTRASONIC_END_ANGLE)
		ultrasonic_motor.reset_angle(0)
		while True:
			go_around_obstacle()

			current_orientation = get_correct_orientation()
			current_robot_distance = robot.distance()
			process_position(current_position, current_orientation, current_robot_distance, prev_robot_distance)
			prev_robot_distance = current_robot_distance
			print("2. current_position[0]:", current_position[0])
			print("2. current_position[1]:", current_position[1])
			print("2. current_orientation:", current_orientation)

			# условие 3 из лекции: невозможность достижения цели
			if (departure_timer.time() > 5000) and (abs(current_position[0] - hit_position[0]) < 10 and abs(current_position[1] - hit_position[1]) < 10):
				print("невозможно достигнуть цели")
				robot.stop()
				# вернуть сонар в исходную ориентацию
				go_back_ultrasonic_sensor()
				return

			# проверить, прошло ли достаточное кол-во времени, чтобы отъехать от линии для обхода препятствия. Также проверить, пересекаем ли прямую линию к цели и текущее расстояние до цели меньше расстояния до цели от точки последнего столкновения
			if departure_timer.time() > 5000 and is_on_line(current_position) and calc_distance_to_goal(current_position) < calc_distance_to_goal(hit_position):
				print("обход препятствия завершен. Продолжаем движение к цели")
				robot.stop()
				print("-1 * gyro_sensor.angle()", -1 * gyro_sensor.angle())
				print("robot.distance()", robot.distance())
				# вернуть робота в исходную ориентацию
				robot.turn(current_orientation - GOAL_ORIENTATION)
				current_orientation = get_correct_orientation()
				print("current_position[0]:", current_position[0])
				print("current_position[1]:", current_position[1])
				print("current_orientation:", current_orientation)
				# вернуть сонар в исходную ориентацию
				go_back_ultrasonic_sensor()
				break
		# end of step 2

	print("2. цель достигнута")
	robot.stop()


# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, 40) # по часовой стрелке
# ultrasonic_motor.run_target(TURN_RATE, 5) # по часовой стрелке
# ultrasonic_motor.run_target(ULTRASONIC_TURN_RATE, -10) # против часовой стрелки
# ultrasonic_motor.run_target(TURN_RATE, -20) # против часовой стрелки


ev3.speaker.beep()
run()
robot.straight(150)
ev3.speaker.beep()
