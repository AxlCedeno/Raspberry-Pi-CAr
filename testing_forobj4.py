import argparse
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import fft, fftfreq
import mod8_func as motor
import math
from picar import PiCar

# parse args
parser = argparse.ArgumentParser()
parser.add_argument('--mock_car', action='store_true')
parser.add_argument('--tim', type=int, default=10)
parser.add_argument("--sampledelay", type=float, default=0.005,help="Delay between samples (seconds)")
parser.add_argument("--calcdelay",type = float, default=0.10, help = "Delay between calculating the motor speed")
parser.add_argument("--loopdelay",type = float, default=1.0, help = "Delay between Starting the Photoresistor Loop")
parser.add_argument("--waitdelay",type = float, default=0.0, help = "Delay before Starting the Motor")
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)

if args.debug:
    print(car)

prev_val = 0
rps = 0
Error = []
AD_reading = []
RPS = []
transitions = []
time_vals = []
differences = []
diff_avg = []
transition_times = []
rpm_vals = []
rps_history = []
transitionsC = 0

def PID_controller(rps, time_elapsed, last_pwm, last_time, kp, Drps, ki, kd):
    error = Drps - rps
    Error.append(error)
    errorT = sum(Error)
    P = kp * error
    I = ki * (errorT) * (time_elapsed - last_time)
    if len(Error) > 1:
        D = kd * ((Error[-1] - Error[-2])/(time_elapsed - last_time))
    else: D = 0

    u_t = P + I + D
    u_tPWM = last_pwm + (4.5 * u_t)

    return u_tPWM    

red = False
green = False
yellow = False

start_time = time.time()
time_elapsed = round(time.time() - start_time,3)

car.set_swivel_servo(0)
car.set_steer_servo(0.1)
car.set_nod_servo(2)

car.set_motor(70)
dist = car.read_distance()

while time_elapsed <= 100.0:
    img = car.get_image()
    if img is not None:
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        cv2.imwrite('TrafficTest.jpg',img)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 150, 150])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 150])
        upper_red2 = np.array([179, 255, 255])
        mask1r = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2r = cv2.inRange(hsv, lower_red2, upper_red2)
        maskRed = mask1r + mask2r
        mask_blurRed = cv2.blur(maskRed,(5,5))
        cv2.imwrite('testingMaskRed.jpg', mask_blurRed)
        threshR = cv2.threshold(mask_blurRed, 200, 255, cv2.THRESH_BINARY)[1]

        maskYellow = cv2.inRange(hsv, (20, 180, 180), (35, 255,255))
        mask_blurYellow = cv2.blur(maskYellow,(5,5))
        cv2.imwrite('testingMaskYellow.jpg',mask_blurYellow)
        threshY = cv2.threshold(mask_blurYellow, 200, 255, cv2.THRESH_BINARY)[1]

        maskGreen = cv2.inRange(hsv, (65, 103, 73), (95, 255, 255))
        mask_blurGreen = cv2.blur(maskGreen,(5,5))
        cv2.imwrite('testingMaskGreen.jpg', mask_blurGreen)
        threshG = cv2.threshold(mask_blurGreen, 200, 255, cv2.THRESH_BINARY)[1]
        cv2.imwrite('testing.jpg', threshG)

        M_R = cv2.moments(threshR)
        M_Y = cv2.moments(threshY)
        M_G = cv2.moments(threshG)

        if (M_R["m00"] != 0.0):
            red = True
            break
        elif (M_Y["m00"] != 0.0):
            yellow = True
            break
        if (M_G["m00"] != 0.0):
            green = True
            break

        time_elapsed = time.time() - start_time

while green:
    print(f'Green: {green}')
    dist = car.read_distance()

    if dist is not None:
        if abs(dist) <= 50:
            car.set_motor(0)
            break

while yellow:
    print(f'Yellow: {yellow}')
    car.set_motor(20)
    dist = car.read_distance()

    if dist is not None:
        if abs(dist) <= 50:
            car.set_motor(0)
            break

while red:
    print(f'Red: {red}')
    dist = car.read_distance()
    
    if dist is not None:
        if abs(dist) <= 300:
            car.set_motor(0)
            break

car.stop()
