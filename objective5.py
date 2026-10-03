import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
from numpy.fft import fft, fftfreq
import mod8_func as motor
from picar import PiCar

# parse args
parser = argparse.ArgumentParser()
parser.add_argument('--mock_car', action='store_true')
parser.add_argument('--tim', type=float, default=100.0)
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

# init car
car = PiCar(mock_car=args.mock_car, threaded=True)

if args.debug:
    print(car)

car.set_nod_servo(0)
time.sleep(1)
car.set_swivel_servo(0)
time.sleep(1)
steer_pos = 0
car.set_steer_servo(0)


start_time = time.time()
time_elapsed = round(time.time() - start_time,3)

pos_steer = 0
pos_swivel = 0

car.set_motor(20)

while (time_elapsed) < args.tim:

    car.set_motor(20)
    dist = car.read_distance()

    if dist is not None:
        if (dist <= 50):
            break

        elif (dist <= 100):
            i = 0
            print('Adjusting')
            car.set_motor(0)
            while (dist <= 100):
                if (i <= 5):
                    pos_swivel = pos_swivel + 2.0
                    car.set_swivel_servo(pos_swivel)
                    time.sleep(0.5)
                    dist = car.read_distance()
                    i = i + 1
                else:
                    if (i == 5):
                        pos_swivel = pos_swivel - 5*(2.0)

                    else: pos_swivel = pos_swivel - 2.0
                    
                    car.set_swivel_servo(pos_swivel)
                    time.sleep(0.5)
                    dist = car.read_distance()
                    i = i + 1
            car.set_steer_servo(2 * pos_swivel)
            car.set_motor(20)
            time.sleep(1.5)
            pos_steer = 0
            pos_swivel = 0
            car.set_steer_servo(pos_steer)
            car.set_swivel_servo(pos_swivel)

    else:
        car.set_motor(0)
    time_elapsed = round(time.time() - start_time,3)

car.set_motor(0)
car.stop()
