import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
import RPi.GPIO as GPIO
import cv2
from dronekit import *
from pymavlink import mavutil
import time
import math
import numpy as np

master = mavutil.mavlink_connection('/dev/ttyACM0')
vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Ждем моторы...")
    time.sleep(1) 

def set_velocity(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def circle(duration):
    vehicle.channels.overrides['1'] = 1200
    time.sleep(duration)
    vehicle.channels.overrides['1'] = 1500

def move_right(duration):
    a = int(1450 + abs(angle) * 2)
    b = int(1700 + abs(angle) * 2)
    vehicle.channels.overrides['3'] = a
    vehicle.channels.overrides['1'] = b
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1200
    vehicle.channels.overrides['1'] = 1500


def move_forward(duration):
    vehicle.channels.overrides['3'] = 1500
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1200


def move_left(duration):
    a = int(1450 + abs(angle) * 2)
    c = int(1350 - abs(angle) * 2)
    vehicle.channels.overrides['3'] = a
    vehicle.channels.overrides['1'] = c
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1200
    vehicle.channels.overrides['1'] = 1500

cap = cv2.VideoCapture(0)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

set_velocity(vehicle, 0.1, 0.1, 0.1)

vehicle.airspeed = 0.1

while True:
    ret, image = cap.read()
    image = cv2.resize(image, (640, 480))
    height, width = image.shape[:2]
    top_cutoff = int(height * 0.01)
    bottom_cutoff = int(height * 0.66)
    image = image[top_cutoff:bottom_cutoff, :]
    theta=0
    minLineLength = 10
    maxLineGap = 20
    lower_blue = np.array([169, 100, 100])
    upper_blue = np.array([189, 255, 255])
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    edges = cv2.Canny(mask, 50, 150)
    cv2.imshow("img", edges)
    if cv2.waitKey(1) >= 0:
        break
    lines = cv2.HoughLinesP(edges,1,np.pi/180,10,minLineLength,maxLineGap)

    if lines is not None:

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            dy = y2 - y1 
            angle_in_radians = math.atan2(dy, dx)
            angle = int(math.degrees(angle_in_radians))

            if angle < 0:
                angle = angle + 90
                print(f"Отклонение от траектории на {angle} градусов")
                if (angle < 10 and angle >= 0) or (angle >= 70 and angle <= 90):
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.HIGH)
                    print("Движение прямо")
                    move_forward(0.5)
                else:
                    GPIO.output(12, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)
                    print("Поворот вправо")
                    move_right(0.5)

            else:
                angle = angle - 90
                print(f"Отклонение от траектории на {-angle} градусов")

                if angle <= -10:
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    print("Поворот влево")
                    move_left(0.5)

                if angle > -10 and angle <= 0:
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.HIGH)
                    print("Движение прямо")
                    move_forward(0.5)
                    
    else:
        print("Линия не найдена")

GPIO.output(12, GPIO.LOW)
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(21, GPIO.LOW)

GPIO.cleanup()
