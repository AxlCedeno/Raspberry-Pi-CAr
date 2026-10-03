import argparse
import time
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from numpy.fft import fft, fftfreq
import mod8_func as motor
from picar import PiCar

# parse args
parser = argparse.ArgumentParser()
parser.add_argument('--mock_car', action='store_true')
parser.add_argument('--timRunning', type=float, default=30.0)
parser.add_argument('--timTracking', type=float, default=30.0)
parser.add_argument('--timC', type=float, default=0.3)
parser.add_argument('--delay', type=float, default=0.5)
parser.add_argument('--delta', type=float, default=0.1)
parser.add_argument('--lorr', type=float, help='Negative for right, positive for left',default=1.0)
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

# init car
car = PiCar(mock_car=args.mock_car, threaded=True)

if args.debug:
    print(car)

start_time = time.time()
elapsed = round(time.time() - start_time,3)

pos = 0
car.set_swivel_servo(pos)
car.set_steer_servo(pos)
time.sleep(0.5)
car.set_motor(40)

distance = []
time_vals = []

start_time = time.time()
elapsed = time.time() - start_time

while(elapsed < args.timRunning):
    dist = car.read_distance()

    if dist is not None:
        distance.append(dist)
        elapsed = time.time() - start_time
        time_vals.append(elapsed)
        if (abs(dist) <= 120):
            print(f'Distance: {dist}')
            car.set_steer_servo(0)
            car.set_motor(0)
            break
            
    img2 = car.get_image()
    if img2 is not None:
        img2 = cv2.cvtColor(img2,cv2.COLOR_RGB2BGR)
        cv2.imwrite('Found2.jpg',img2)

        hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (105, 70, 70), (130, 255,255))
        cv2.imwrite('testingMask.jpg',mask)
        mask_blur = cv2.blur(mask,(5,5))
        thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_BINARY)[1]
        cv2.imwrite('testing.jpg', thresh)
    
        #Getting the Center of Mass
        M = cv2.moments(thresh)

        if (M["m00"] == 0):
            pos = pos + 5.0 * (args.lorr)
            car.set_steer_servo(pos)
            time.sleep(args.timC)

        else:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            h, w = img2.shape[:2]
            cx = w / 2
            cy = h / 2
        
            if abs(cx - cX) < 0.05:
                pos = 1.0
                car.set_steer_servo(pos)
        
            # elif abs(cy - cY) < 0.05:
            #   pos = 0.05
            #    car.set_steer_servo(pos)

            else:
                theta = math.degrees(math.atan2((cX - cx), (cY - cy)))
        
                #dc = dc - 0.5
                pos = pos - (args.delta * theta * (0.0527))
                car.set_steer_servo(pos)

    else: car.set_steer_servo(0)

    elapsed = round(time.time() - start_time,3)

fname = 'sample.txt'
with open(fname, 'w') as file:
    for i in range(len(distance)):
        file.write(f'Time: {time_vals[i]: 3f}\tDistance: {distance[i]: 3f}\n')

car.set_motor(0)
time.sleep(1)
car.stop()
