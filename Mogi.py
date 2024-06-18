#!/usr/bin/env pybricks-micropython
from threading import Thread
from time import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor,UltrasonicSensor,TouchSensor)
from pybricks.parameters import (Port,Stop, Direction, Align, Color)
from pybricks.robotics import DriveBase
from pybricks.tools import print, wait, StopWatch
import random
from random import *
ev3 = EV3Brick()

# Definir os motores
motor_R = Motor(Port.A)
motor_L = Motor(Port.B)
arm_L = Motor(Port.C)
arm_R = Motor(Port.D)

# Definir os sensores
sensor_color = ColorSensor(Port.S1)
sensor_L = UltrasonicSensor(Port.S2)
sensor_R = UltrasonicSensor(Port.S3)
touch = TouchSensor(Port.S4)

#definindo variáveis 
SPEED = 100 ; ENEMY_DISTANCE = 600  ; EDGE = 5
color_ground = 0 ; color_ground = sensor_color.reflection(); timer = StopWatch(); 
random.seed(int(time()*1000)); line_detected = False; turn = 0; turn = random.randint(0,1)

ev3.speaker.set_volume(volume=100, which="_all_")
ev3.speaker.set_speech_options(language="pt-pt", voice="whisperf")
ev3.speaker.say("LE GO LO MA NI A CO")
ev3.speaker.beep(frequency=1000, duration=100)

while touch.pressed() == False:
    pass
for i in range(5):
        wait(900) ; ev3.speaker.beep(frequency=1000, duration=100)

def check_boundaries():
    global line_detected
    while True:
        value_ground = sensor_color.reflection()
        if abs(value_ground - color_ground) >=  EDGE_THRESHOLD:
            line_detected = True
            ev3.light.on(Color.YELLOW)
            motor_L.dc(SPEED)
            motor_R.dc(SPEED)
            wait(500)
            line_detected = False
            motor_L.stop(Stop.BRAKE)
            motor_R.stop(Stop.BRAKE)
        wait(10)

#Thread(target=open_arms).start() -- fazer o abrir braços
#arm_L.run_until_stalled(speed=500, then=Stop.HOLD, duty_limit=50); arm_R.run_until_stalled(speed=500, then=Stop.HOLD, duty_limit=50)
#int angle_L = arm_L.angle(); int angle_R = arm_R.angle(); print("arm_L",angle_L); print("arm_R,angle_R")

Thread(target=check_boundaries).start()
motor_L.dc(-SPEED)
motor_R.dc(-SPEED)
while line_detected:
    pass
while True:
    #ev3.screen.load_image(ANGRY) ##VER SE A CARINHA FUNCIONA
    if line_detected == False:
        timer.reset()
        motor_L.dc(SPEED*turn)
        motor_R.dc(SPEED*int(not turn))
        ev3.light.on(color.GREEN)
    while timer.time() < random.randint(1, 3)*2000 and line_detected == False:
        if sensor_L.distance() <= ENEMY_DISTANCE and sensor_R.distance() <= ENEMY_DISTANCE:
            motor_L.dc(-SPEED)
            motor_R.dc(-SPEED)
            ev3.light.on(Color.RED)
            while sensor_L.distance() <= ENEMY_DISTANCE and sensor_R.distance() <= ENEMY_DISTANCE:
                wait(10)
        elif sensor_L.distance() <= ENEMY_DISTANCE or sensor_R.distance() <= ENEMY_DISTANCE:
            ev3.light.on(Color.RED)
            if sensor_L.distance() < sensor_R.distance():
                motor_L.dc(-80); motor_R.dc(SPEED)
            else:
                motor_L.dc(SPEED); motor_R.dc(-80)
        wait(10)
    turn = int(not turn)
    motor_L.dc(-SPEED)
    motor_R.dc(-SPEED)
    wait(10)
def attack():
    while True:
        if line_detected:
            global timer
            timer = time.time()
            motor_L.on(SPEED, SPEED)
            motor_R.on(SPEED, SPEED)
        while time.time() - timer < random.randint(1, 3)*2:
            if ((sensor_L.distance_centimeters + sensor_R.distance_centimeters)/2) <= ENEMY_DISTANCE:
                SPEED = 100
                motor_L.on(-SPEED_RUN, -SPEED_RUN)
                motor_R.on(-SPEED_RUN, -SPEED_RUN)
                sound.speak("Attack!")
                while ((sensor_L.distance_centimeters + sensor_R.distance_centimeters)/2) <= ENEMY_DISTANCE:
                    time.sleep(0.01)
        time.sleep(0.01)

check_boundaries_thread = threading.Thread(target=check_boundaries)
check_boundaries_thread.daemon = True
check_boundaries_thread.start()

attack_thread = threading.Thread(target=attack)
attack_thread.daemon = True
attack_thread.start()

while True:
    time.sleep(0.01)