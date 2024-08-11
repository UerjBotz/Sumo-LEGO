#!/usr/bin/env python3

# Import the necessary libraries

import time
import math

from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# # Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)


spkr = Sound()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2) #esquerda
gyro_sensor_in3 = GyroSensor(INPUT_3)
##outro ultra add aqui no futuro com INPUT_6 q é o da direita

# Here is where your code starts

def to_vendo_inimigo():
    #raio do dohyo simulado = 100cm
    if ultrasonic_sensor_in2.distance_centimeters < 200:
        return True
    return False


class Estado:
    PROCURANDO = 1
    ATACANDO = 2
    AFASTANDO = 3
    

#constantes
STATUS_INICIAL = Estado.PROCURANDO
VEL_CORRE = 100 #porcentagem
VEL_PROCURA = 40 

COR_ARENA = 6 #branco
COR_BORDA = 5 #vermelho

status = STATUS_INICIAL
while True:
    
    #simulador: pista branca, borda vermelha
    
    #define estado (procurando, atacando, naocair)
    if status == Estado.PROCURANDO:
        if color_sensor_in1.color == COR_ARENA: 
            if to_vendo_inimigo():
                tank_drive.stop()
                time.sleep(0.1)
                status = Estado.ATACANDO
        elif color_sensor_in1.color == COR_BORDA:
            tank_drive.stop()
            status = Estado.AFASTANDO
        
    elif status == Estado.ATACANDO:
        if color_sensor_in1.color == COR_ARENA:
            if not to_vendo_inimigo():
                tank_drive.stop()
                status = Estado.PROCURANDO
        elif color_sensor_in1.color == COR_BORDA:
            tank_drive.stop()
            status = Estado.AFASTANDO
        
    elif status == Estado.AFASTANDO:
        if color_sensor_in1.color == COR_ARENA:
            tank_drive.stop()
            status = Estado.PROCURANDO
    
    #define ação
    if status == Estado.PROCURANDO:
        tank_drive.on(VEL_PROCURA, -VEL_PROCURA)

        
    elif status == Estado.ATACANDO:
        tank_drive.on(VEL_CORRE, VEL_CORRE)

        
    elif status == Estado.AFASTANDO:
        tank_drive.on(-VEL_CORRE, -VEL_CORRE)
