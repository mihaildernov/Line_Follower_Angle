import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import cv2
import picamera
from dronekit import *
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0')
vehicle = connect('/dev/ttyACM0', wait_ready=True)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Ждем моторы...")
    time.sleep(1) 

def circle(duration):
    vehicle.channels.overrides['1'] = 1200
    time.sleep(duration)
    vehicle.channels.overrides['1'] = 1500

def move_right(duration):
    a = int(1400 + angle * 5)
    b = int(1700 + angle * 5)
    vehicle.channels.overrides['3'] = a
    vehicle.channels.overrides['1'] = b
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1400
    vehicle.channels.overrides['1'] = 1500

def move_forward(duration):
    vehicle.channels.overrides['3'] = 1500
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1400

def move_left(duration):
    a = int(1400 + angle * 5)
    c = int(1350 - angle * 5)
    vehicle.channels.overrides['3'] = a
    vehicle.channels.overrides['1'] = c
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1400
    vehicle.channels.overrides['1'] = 1500

cap = cv2.VideoCapture(0)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

while True:
    ret, image = cap.read()
    image = cv2.resize(image, (640, 480))
    height, width = image.shape[:2]
    top_cutoff = int(height * 0.25)
    bottom_cutoff = int(height * 0.75)
    image = image[top_cutoff:bottom_cutoff, :]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, math.pi / 180, 200)

    if lines is not None:

        for line in lines:
            rho, theta = line[0]
            angle = int(math.degrees(theta))

            if angle > 120:
                angle = angle - 90
                angle2 = -angle
                print(f"Отклонение от траектории на {-angle} градусов")
            else:
                angle2 = angle
                print(f"Отклонение от траектории на {angle} градусов")

                if angle2 > 10:
                    GPIO.output(12, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)
                    print("Поворот вправо")
                    move_right(0.5)

                elif angle2 < 10 and angle2 > -10:
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.HIGH)
                    print("Движение прямо")
                    move_forward(0.5)

                elif angle2 < -10:
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    print("Поворот влево")
                    move_left(0.5)
                
                else:
                    print("Линия не найдена")

    cv2.imshow("img", image)
    if cv2.waitKey(1) >= 0:
        break

GPIO.output(12, GPIO.LOW)
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(21, GPIO.LOW)

GPIO.cleanup()
