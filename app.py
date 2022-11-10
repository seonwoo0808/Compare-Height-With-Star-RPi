import pigpio

import picamera
import numpy as np
import cv2
import time
import threading
import spidev
import asyncio
import websockets
import os
import dotenv
from pickle import dumps, loads

dotenv.load_dotenv()

# Hyperparameters

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3 

SERVO_PIN = 13

BUTTON_PIN = 16

TRIG = 17
ECHO = 4
A=5
B=6
C=12
D=16
E=26
F=20
G=21

d1=25
d2=22
d3=24
d4=23

# Global variables

captured_arr = np.empty((480, 640, 3), dtype=np.uint8)
resized_arr = np.empty((480, 640,), dtype=np.uint8)

distance = 0
prev_value = 0
degree = 0
button_pushed = False

# Set up the thread
def camera_thread():
    with picamera.PiCamera() as camera:
        while True:
            try:
                camera.resolution = (640, 480)
                camera.framerate = 24
                time.sleep(0.05)
                camera.capture(captured_arr, 'rgb')
                resized_arr = cv2.cvtColor(captured_arr, cv2.COLOR_RGB2GRAY)
            except KeyboardInterrupt:
                break

def sonic():
   sonic_timer_start = time.time()
   
   pi.write(TRIG, False)
   time.sleep(0.01)
   pi.write(TRIG, True)
   time.sleep(0.00001)
   pi.write(TRIG, False)
   while pi.read(ECHO)==0:
      sonic_timer_start = time.time()
   while pi.read(ECHO)==1:
      sonic_timer_stop = time.time()
   sonic_time_elapsed = sonic_timer_start - sonic_timer_stop
   distance = int(sonic_time_elapsed * 17160.0)

def controll():
    prev_value=0
    button_pushed = False
    while True:
        try:
            analog_value = analog_read(0)*1.9 + 500
            if abs(analog_value - prev_value) > 50:
                prev_value = analog_value
                pi.set_servo_pulsewidth(SERVO_PIN, analog_value)
                degree = int((analog_value -500)*0.09)
            button_read = pi.read(BUTTON_PIN)
            if button_pushed == False and button_read == 1:
                button_pushed = True
                if top["degree"] == None:
                    top["degree"] = degree
                    top["distance"] = distance
                elif bottom["degree"] == None:
                    bottom["degree"] = degree
                    bottom["distance"] = distance
            elif button_pushed == True and button_read == 0:
                button_pushed = False
            time.sleep(0.01)
        except KeyboardInterrupt:
            
            break

async def connect():

    # 웹 소켓에 접속을 합니다.
    async with websockets.connect("ws://"+os.environ["HOST"]+":8000/rpi/1") as websocket:

        await websocket.send("Raspberry Pi Hello")
        data = await websocket.recv()
        if data == "Web Server Hello":
            
            while True:
                coord = np.array([degree, distance], dtype=np.float32)
                await websocket.send(dumps(coord))
                data = await websocket.recv()
                top_and_bottom = np.array([], dtype=np.float32)
                await websocket.send(dumps(top_and_bottom))
                data = await websocket.recv()
                if data == "OK":
                    await websocket.send(dumps(resized_arr))
                    data = await websocket.recv()
                    if data == "OK":
                        continue
                break

# Set up the GPIO
pi = pigpio.pi()

# Set up the SPI
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000


# Button Initialization
pi.set_mode(BUTTON_PIN, pigpio.INPUT)
pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_DOWN)


# Ultrasonic Initialization
pi.set_mode(TRIG, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)


def analog_read(channel):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    adc_out = ((r[1]&3) << 8) + r[2]
    return adc_out



cam_thread_obj = threading.Thread(target=camera_thread, args=(), daemon=True)
cam_thread_obj.start()
sonic_thread_obj = threading.Thread(target=sonic, args=(), daemon=True)
sonic_thread_obj.start()
controlle_thread_obj = threading.Thread(target=controll, args=(), daemon=True)
controlle_thread_obj.start()

top = {"degree": None, "distance": None}
bottom = {"degree": None, "distance": None}


asyncio.get_event_loop().run_until_complete(connect())