#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Image, ImageFile
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import (
    Port, 
    Stop, 
    Direction, 
    Color)
from pybricks.tools import print, wait, StopWatch

from threading import Thread
import random
from time import time

EDGE_THRESHOLD = 20  # Increased to better detect black vs white
ENEMY_DISTANCE = 100
SPEED_RUN = 100  # Power %
GROUND_COLOR_BLACK = 10  # Example reflection value for black
GROUND_COLOR_WHITE = 85  # Example reflection value for white

ev3 = EV3Brick()

def countdown():
    # Wait 5 seconds, as per DA RULEZ
    for i in range(5):
        wait(900)
        ev3.speaker.beep(frequency=1000, duration=100)

def lower_wedge():
    # Lower the wedge while the other stuff is initialising
    motor_W.run_until_stalled(speed=500, then=Stop.HOLD, duty_limit=50)
    # Lift the wedge just a bit to avoid dragging it on the floor and losing traction
    motor_W.run_angle(speed=500, rotation_angle=-10, then=Stop.HOLD, wait=False)

def engage_traction():
    motor_W.run(SPEED_RUN)

def check_boundaries():
    global roam_enabled
    while True:
        value_L = sensor_L.reflection()
        if value_L >= GROUND_COLOR_WHITE - EDGE_THRESHOLD:
            roam_enabled = False  # Stop the other behaviours
            ev3.light.on(Color.YELLOW)
            # Back away from the edge
            motor_L.dc(SPEED_RUN)
            motor_R.dc(SPEED_RUN)
            wait(100)
            motor_L.stop(Stop.BRAKE)
            motor_R.stop(Stop.BRAKE)
            # Resume the other behaviours
            roam_enabled = True
        wait(10)

def main_behavior():
    global roam_enabled, turn_right
    while True:
        if roam_enabled:
            timer.reset()
            motor_L.run(SPEED_RUN)
            motor_R.run(-SPEED_RUN)
            ev3.light.on(Color.GREEN)
            while timer.time() < random.randint(1, 3) * 2000 and roam_enabled:
                # Check for enemy presence
                if sensor_D.distance() <= ENEMY_DISTANCE:
                    engage_traction()
                    motor_L.run(SPEED_RUN)
                    motor_R.run(-SPEED_RUN)
                    ev3.light.on(Color.RED)
                elif sensor_D.distance() <= 50:
                    turn_right()
                    ev3.light.on(Color.RED)
                elif sensor_D.distance() <= 150:
                    turn_left()
                    ev3.light.on(Color.RED)
                wait(10)
            turn_right = int(not turn_right)  # Now turn the other way
            wait(10)

def color_based_movement():
    while True:
        reflection = sensor_L.reflection()
        if reflection <= GROUND_COLOR_BLACK + EDGE_THRESHOLD:
            motor_L.run(SPEED_RUN)
            motor_R.run(-SPEED_RUN)
        elif reflection >= GROUND_COLOR_WHITE - EDGE_THRESHOLD:
            motor_L.stop(Stop.BRAKE)
            motor_R.stop(Stop.BRAKE)
            wait(100)
            motor_L.run(-SPEED_RUN)
            motor_R.run(SPEED_RUN)
            wait(100)
            motor_L.run(SPEED_RUN)
            motor_R.run(-SPEED_RUN)
        wait(10)

# MODIFICAÇÃO: Função para virar à direita
def turn_right():
    motor_L.run_angle(speed=500, rotation_angle=90, then=Stop.HOLD, wait=True)
    motor_R.run_angle(speed=500, rotation_angle=-90, then=Stop.HOLD, wait=True)

# MODIFICAÇÃO: Função para virar à esquerda
def turn_left():
    motor_L.run_angle(speed=500, rotation_angle=-90, then=Stop.HOLD, wait=True)
    motor_R.run_angle(speed=500, rotation_angle=90, then=Stop.HOLD, wait=True)

# Initialisation
motor_L = Motor(Port.B, Direction.CLOCKWISE) # Esquerda
motor_R = Motor(Port.A, Direction.CLOCKWISE) # Direita
motor_W = Motor(Port.C, Direction.COUNTERCLOCKWISE) # Tração (Wedge)
sensor_L = ColorSensor(Port.S1) # Sensor de cor esquerda
sensor_D = UltrasonicSensor(Port.S2) # Sensor ultrassônico
timer = StopWatch()
ground_colour = 0
turn_right = 0
velRotate = SPEED_RUN * turn_right
velRotateother = - (SPEED_RUN * int(not turn_right))
roam_enabled = True  # Flag global para habilitar/desabilitar o comportamento de roaming
random.seed(int(time() * 1000))
turn_right = random.randint(0, 1)
ev3.speaker.set_volume(volume=100, which="_all_")
ev3.speaker.set_speech_options(language="pt-pt", voice="whisperf")
ev3.speaker.say("LE GO LO MA NI A CO")

# Wait for a hand to be placed in front of the sensor, then removed, to start the initial countdown
ev3.speaker.beep(frequency=1000, duration=100)
while sensor_D.distance() > 50:
    pass
ev3.speaker.beep(frequency=1000, duration=100)
while sensor_D.distance() < 50:
    pass
countdown()

# Match begin: run towards the edge before turning around
motor_W.run_angle(speed=500, rotation_angle=135, then=Stop.HOLD, wait=False)
Thread(target=lower_wedge).start()
Thread(target=check_boundaries).start()  # Begin edge detection
Thread(target=main_behavior).start()  # Start the main behavior
Thread(target=color_based_movement).start()  # Start color based movement

motor_L.dc(-SPEED_RUN)
motor_R.dc(-SPEED_RUN)
while roam_enabled == False:
    pass  # Wait for the edge detection to detect the edge before starting the main behaviour
