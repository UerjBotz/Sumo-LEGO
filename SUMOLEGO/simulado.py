#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_esquerda = UltrasonicSensor(INPUT_2) #esquerda
ultrasonic_sensor_direita = UltrasonicSensor(INPUT_6)  #direita
gyro_sensor_in3 = GyroSensor(INPUT_3)

# Here is where your code starts

def to_vendo_inimigo():
    #raio do dohyo simulado = 100cm
    distancia_esquerda = ultrasonic_sensor_esquerda.distance_centimeters
    distancia_direita = ultrasonic_sensor_direita.distance_centimeters
    
    if distancia_esquerda < 200 or distancia_direita < 200:
        return True, distancia_esquerda, distancia_direita
    return False, distancia_esquerda, distancia_direita


class Estado:
    PROCURANDO_ESQUERDA = 1
    PROCURANDO_DIREITA = 2
    ATACANDO = 3
    AFASTANDO = 4
    

#constantes
STATUS_INICIAL = Estado.PROCURANDO_ESQUERDA
VEL_CORRE = 85 #porcentagem
VEL_PROCURA = 40 

COR_ARENA = 6 #branco
COR_BORDA = 5 #vermelho

status = STATUS_INICIAL

while True:
    
    #simulador: pista branca, borda vermelha
    
    #define estado (procurando-dir, procurando-esq, atacando, naocair)
    if status == Estado.PROCURANDO_ESQUERDA:
        if color_sensor_in1.color == COR_ARENA: 
            vendo_inimigo, dist_esq, dist_dir = to_vendo_inimigo()
            if vendo_inimigo:
                tank_drive.stop()
                time.sleep(0.1)
                status = Estado.ATACANDO
            elif dist_dir < dist_esq:
                # Se o inimigo é detectado pela direita, muda para procurar à direita
                tank_drive.stop()
                time.sleep(0.1)
                status = Estado.PROCURANDO_DIREITA
        elif color_sensor_in1.color == COR_BORDA:
            tank_drive.stop()
            status = Estado.AFASTANDO
        
    elif status == Estado.PROCURANDO_DIREITA:
        if color_sensor_in1.color == COR_ARENA: 
            vendo_inimigo, dist_esq, dist_dir = to_vendo_inimigo()
            if vendo_inimigo:
                tank_drive.stop()
                time.sleep(0.1)
                status = Estado.ATACANDO
            elif dist_esq < dist_dir:
                # Se o inimigo é detectado pela esquerda, muda para procurar à esquerda
                tank_drive.stop()
                time.sleep(0.1)
                status = Estado.PROCURANDO_ESQUERDA
        elif color_sensor_in1.color == COR_BORDA:
            tank_drive.stop()
            status = Estado.AFASTANDO

    elif status == Estado.ATACANDO:
        if color_sensor_in1.color == COR_ARENA:
            vendo_inimigo, dist_esq, dist_dir = to_vendo_inimigo()
            if not vendo_inimigo:
                tank_drive.stop()
                status = Estado.PROCURANDO_ESQUERDA
        elif color_sensor_in1.color == COR_BORDA:
            tank_drive.stop()
            status = Estado.AFASTANDO
        
    elif status == Estado.AFASTANDO:
        if color_sensor_in1.color == COR_ARENA:
            tank_drive.stop()
            status = Estado.PROCURANDO_ESQUERDA
    
    #define ação
    if status == Estado.PROCURANDO_ESQUERDA:
        # Gira para a esquerda
        tank_drive.on(-VEL_PROCURA, VEL_PROCURA)

        
    elif status == Estado.PROCURANDO_DIREITA:
        # Gira para a direita
        tank_drive.on(VEL_PROCURA, -VEL_PROCURA)

        
    elif status == Estado.ATACANDO:
        tank_drive.on(VEL_CORRE, VEL_CORRE)

        
    elif status == Estado.AFASTANDO:
        tank_drive.on(-VEL_CORRE, -VEL_CORRE)
