import sys
sys.path.append("/usr/local/lib/python3.9/site-packages")
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import cv2
import picamera
from dronekit import *
from pymavlink import mavutil
import time
import math

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
    vehicle.channels.overrides['3'] = a
    vehicle.channels.overrides['1'] = b
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1100
    vehicle.channels.overrides['1'] = 1500

def move_forward(duration):
    vehicle.channels.overrides['3'] = 1500
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1100

def move_left(duration):
    vehicle.channels.overrides['3'] = a
    vehicle.channels.overrides['1'] = c
    time.sleep(duration)
    vehicle.channels.overrides['3'] = 1100
    vehicle.channels.overrides['1'] = 1500

camera = picamera.PiCamera()
camera.framerate = 20
camera.zoom = (0.1, 0.1, 0.6, 0.1)
rawCapture = PiRGBArray(camera,size=(1920,1080))

time.sleep(1)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    cv2.imshow('img',image)
    key = cv2.waitKey(1) & 0xFF
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(edges, 1, math.pi / 180, 200)
    
    if lines.any():
        
        for line in lines:
            rho, theta = line[0]
            angle = math.degrees(theta)

            if angle > 120:
                angle = angle - 90
                angle2 = -angle
                print("Угол между линией и центром:", -angle)
            else:
                angle2 = angle
                print("Угол между линией и центром:", angle)

                a = 1400 + angle * 1,5
                b = 1700 + angle * 1,5
                c = 1350 - angle * 1,5

                if angle2 > -2:
                    GPIO.output(12, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)
                    print("Поворот вправо")
                    move_right(0.5)

                if angle2 < 2 and angle2 > -2:
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.HIGH)
                    print("Движение прямо")
                    move_forward(0.5)

                if angle2 < 2:
                    GPIO.output(12, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    print("Поворот влево")
                    move_left(0.5)
                    
    if key == ord("q"):
            break

rawCapture.truncate(0)

GPIO.output(12, GPIO.LOW)
GPIO.output(16, GPIO.LOW)
GPIO.output(20, GPIO.LOW)
GPIO.output(21, GPIO.LOW)

GPIO.cleanup()
