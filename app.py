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



SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3 

SERVO_PIN = 13

BUTTON_PIN = 16

captured_arr = np.empty((480, 640, 3), dtype=np.uint8)
resized_arr = np.empty((480, 640,), dtype=np.uint8)
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

async def connect():

    # 웹 소켓에 접속을 합니다.
    async with websockets.connect("ws://"+os.environ["HOST"]+":8000/rpi") as websocket:

        await websocket.send("Raspberry Pi Hello")
        data = await websocket.recv()
        if data == "Web Server Hello":
            
            while True:
                coord = np.array([degree], dtype=np.float32)
                await websocket.send(dumps(coord))
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



def analog_read(channel):
    r = spi.xfer2([1, (8 + channel) << 4, 0])
    adc_out = ((r[1]&3) << 8) + r[2]
    return adc_out

prev_value = 0
degree = 0
button_pushed = False

cam_thread_obj = threading.Thread(target=camera_thread, args=(), daemon=True)
cam_thread_obj.start()
asyncio.get_event_loop().run_until_complete(connect())

while True:
    try:
        analog_value = analog_read(0)*1.9 + 500
        if abs(analog_value - prev_value) > 50:
            prev_value = analog_value
            print(analog_value)
            pi.set_servo_pulsewidth(SERVO_PIN, analog_value)
            degree = int((analog_value -500)*0.09)
        button_read = pi.read(BUTTON_PIN)
        if button_pushed == False and button_read == 1:
            button_pushed = True
            print("Button Pushed")
            cv2.imwrite('image.jpg', output)
            print("Image Saved")
        elif button_pushed == True and button_read == 0:
            button_pushed = False
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        cam_thread_obj.stop()
        break

