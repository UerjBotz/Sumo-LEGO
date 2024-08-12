#!/usr/bin/env pybricks-micropython
# import os
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Image, ImageFile
from pybricks.ev3devices import (Motor, ColorSensor, InfraredSensor, UltrasonicSensor)
from pybricks.parameters import (
    Port, 
    Stop, 
    Direction, 
    Align, 
    Color)
from pybricks.tools import print, wait, StopWatch

from threading import Thread
import random
from time import time

EDGE_THRESHOLD = 5
ENEMY_DISTANCE =  370
SPEED_RUN = 100  # Power 
SPEED_SEARCH = 900  # Deg/s

ev3 = EV3Brick()

def countdown():
    # Contagem de 5 segundos obrigatória antes de começar a luta
    for i in range(5):
        wait(900)
        ev3.speaker.beep(frequency=1000, duration=100)

def checarBordas():
    # Continuously check if the boundary is reached, and, if so, blocks the other behaviours and moves away from it
    
    global andar
    if sensor_enemy.distance() <= ENEMY_DISTANCE:
        andar = True
    
    print("base colour", cor_chao)
    while True:
        #value_L = sensor_cor.reflection()
        #print("Color detected!", cor_chao, value_L)
        if value_L >= (cor_chao + EDGE_THRESHOLD):
            andar = False  # Stop the other behaviours
            print("Color detected!", cor_chao, value_L)
            ev3.light.on(Color.YELLOW)

            # Stop the motors
            motor_L.stop(Stop.BRAKE)
            motor_R.stop(Stop.BRAKE)

            # Reverse
            motor_L.run_time(speed=-SPEED_RUN, time=500)
            motor_R.run_time(speed=-SPEED_RUN, time=500)
            motor_tracao.run_time(speed=-SPEED_RUN, time=500)

            # Turn right
            motor_L.run_time(speed=SPEED_RUN, time=1000)
            motor_R.run_time(speed=-SPEED_RUN, time=1000)

            # Resume the other behaviours
            andar = True
            print("Roaming again")
        # if statemenr if it detects white color on the floor then stop the robot, turn reverse and turn right (faster)

        wait(10)


# Initialisation
motor_L = Motor(Port.B, Direction.CLOCKWISE)#Esquerda
motor_R = Motor(Port.A, Direction.CLOCKWISE)#Direita
motor_tracao = Motor(Port.C, Direction.COUNTERCLOCKWISE)
timer = StopWatch()
# sensor_cor = ColorSensor(Port.S1) # ESQUERDA
sensor_enemy = UltrasonicSensor(Port.S2) #Ultrasonico # Esquerda

cor_chao = 0
andar = True  # Global flag to enable/disable the roaming behaviour

random.seed(int(time()*1000))
turn_right = random.randint(0, 1)

# InfraRed Sensor by judge's request
# State 1: pronto
# State 2: start
# State 3: stop


print("Initial turn right", turn_right)
# cor_chao = sensor_cor.reflection()
# ev3.speaker.set_volume(volume=100, which="_all_")
# ev3.speaker.set_speech_options(language="pt-pt", voice="whisperf")
# ev3.speaker.say("LE GO LO MA NI A CO")

# Main behaviour
while True:
    print("Velocidade", SPEED_RUN)
    # if sensor_cor.reflection() <= cor_chao:
    #     andar = True
    if andar:
        timer.reset()
        motor_L.dc(SPEED_RUN)
        motor_R.dc(-SPEED_RUN)
        motor_tracao.dc(SPEED_RUN)
        ev3.light.on(Color.GREEN)
        while timer.time() < random.randint(1, 3)*2000 and andar:
            if sensor_enemy.distance() <= ENEMY_DISTANCE:
                print("ENEMY!", sensor_enemy.distance())
                # Enemy found: full speed ahead
                motor_L.dc(SPEED_RUN) # sentido antihorario
                motor_R.dc(SPEED_RUN)
                motor_tracao.dc(SPEED_RUN)
                ev3.light.on(Color.RED)
            # if stopped dectecting the enemy, stop the motors
            if sensor_enemy.distance() > ENEMY_DISTANCE: #isso aqui tá errado (e é provavelmente inútil)
                motor_L.stop(Stop.BRAKE)
                motor_R.stop(Stop.BRAKE)
                motor_tracao.stop(Stop.BRAKE)
                ev3.light.off()

            # search for the enemy if it's not in front of the robot, increase the time of rotation
            if sensor_enemy.distance() > ENEMY_DISTANCE:
                motor_L.dc(SPEED_SEARCH*turn_right)
                motor_R.dc(-SPEED_SEARCH*turn_right)
                #motor_tracao.dc(SPEED_RUN) #colocar aqui pra ele soltar o motor
                ev3.light.on(Color.ORANGE)

            wait(10)
        turn_right = int(not turn_right)  # Now turn the other way
        wait(10)