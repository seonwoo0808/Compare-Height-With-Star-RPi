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
import base64
from pickle import dumps, loads

dotenv.load_dotenv()

# Hyperparameters

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3 

SERVO_PIN = 13

BUTTON_PIN = 16

TRIG = 17
ECHO = 4

SEGMENT_PINS = [5, 6, 12, 16, 26, 20, 21]
DIGIT_PINS = [25, 22, 24, 23]

data = [[1, 1, 1, 1, 1, 1, 0],  #0
        [0, 1, 1, 0, 0, 0, 0],  #1
        [1, 1, 0, 1, 1, 0, 1],  #2
        [1, 1, 1, 1, 0, 0, 1],  #3
        [0, 1, 1, 0, 0, 1, 1],  #4
        [1, 0, 1, 1, 0, 1, 1],  #5
        [1, 0, 1, 1, 1, 1, 1],  #6
        [1, 1, 1, 0, 0, 0, 0],  #7
        [1, 1, 1, 1, 1, 1, 1],  #8
        [1, 1, 1, 0, 0, 1, 1]]  #9

# Global variables

im_b64 = None

distance = 0
prev_value = 0
degree = 0
button_pushed = False

height = 0

top = {"degree": 0, "distance": 0} # top degree, distance
bottom = {"degree": 0, "distance": 0} # bottom degree, distance

# Set up the thread
def camera_thread(): # camera thread
    global im_b64
    with picamera.PiCamera() as camera:
        captured_arr = np.empty((480, 640, 3), dtype=np.uint8)
        while True:
            try:
                camera.resolution = (640, 480)
                camera.framerate = 24
                time.sleep(0.05)
                camera.capture(captured_arr, 'rgb')
                # resize to 320x240
                _, im_arr = cv2.imencode('.jpg', captured_arr)  # im_arr: image in Numpy one-dim array format.
                im_bytes = im_arr.tobytes()
                im_b64 = base64.b64encode(im_bytes).decode('utf-8')
            except KeyboardInterrupt:
                break

def sonic():
    global distance
    while True:
        pi.write(TRIG, True)
        time.sleep(0.00001)
        pi.write(TRIG, False)
        start = time.time()
        i=0
        while pi.read(ECHO)==0:
            start = time.time()
            i+=1
            if i>10000:
                break
        while pi.read(ECHO)==1:
            stop = time.time()
        sonic_time_elapsed = stop - start
        distance = sonic_time_elapsed * 17160
        time.sleep(0.01)
            

            
def controll():
    global degree
    global height
    prev_value=0
    button_pushed = False
    flag = False
    bottom_height = 0
    top_height = 0
    height=0
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
                if flag == False:
                    flag = True
                    top["degree"] = degree
                    top["distance"] = distance
                    radian = (90 - degree) * 3.14 / 180
                    top_height = distance * np.cos(radian)
                else:
                    flag = False
                    bottom["degree"] = degree
                    bottom["distance"] = distance
                    radian = (90 - degree) * 3.14 / 180
                    bottom_height = distance * np.cos(radian) 
                height = top_height + bottom_height
            elif button_pushed == True and button_read == 0:
                button_pushed = False
            print(height)
            time.sleep(0.01)
        except KeyboardInterrupt:
            
            break

def display():  #자리수, 숫자
    while True:
        number_str = "{:04d}".format(int(height))
        
        for digit, number in enumerate(number_str):
            for i in range(len(DIGIT_PINS)):
                    if i+1 == digit:
                        pi.write(DIGIT_PINS[i], False)
                    else:
                        pi.write(DIGIT_PINS[i], True)
            # 숫자 출력
            for i in range(len(SEGMENT_PINS)):
                    pi.write(SEGMENT_PINS[i], data[int(number)][i])
            time.sleep(0.0001)

async def connect():

    # 웹 소켓에 접속을 합니다.
    async with websockets.connect("ws://"+os.environ["HOST"]+":8000/rpi/1") as websocket:

        await websocket.send("Raspberry Pi Hello")
        data = await websocket.recv()
        if data == "Web Server Hello":
            
            while True:
                coord = str(degree)+","+str(distance)
                await websocket.send(coord)
                data = await websocket.recv()
                await websocket.send("{},{},{},{},{}".format(top["degree"], top["distance"], bottom["degree"], bottom["distance"], height))
                data = await websocket.recv()
                if data == "OK":
                    while im_b64 == None:
                        print("Waiting for image")
                        time.sleep(0.5)
                    await websocket.send(im_b64)
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
pi.write(TRIG, False)
time.sleep(0.01)

# 4 Digit 7 Segment Initialization
for segment in SEGMENT_PINS:
    pi.set_mode(segment, pigpio.OUTPUT)
    pi.write(segment, False)

for digit in DIGIT_PINS:
    pi.set_mode(digit, pigpio.OUTPUT)
    pi.write(digit, True)

def analog_read(channel): # read ADC value
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    adc_out = ((r[1]&3) << 8) + r[2]
    return adc_out


# create thread object and run

cam_thread_obj = threading.Thread(target=camera_thread, args=(), daemon=True)
cam_thread_obj.start()
sonic_thread_obj = threading.Thread(target=sonic, args=(), daemon=True)
sonic_thread_obj.start()
controlle_thread_obj = threading.Thread(target=controll, args=(), daemon=True)
controlle_thread_obj.start()
fnd_thread_obj = threading.Thread(target=display, args=(), daemon=True)
fnd_thread_obj.start()



asyncio.get_event_loop().run_until_complete(connect())