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


img = car.get_image()
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

if (M_Y["m00"] != 0.0):
    yellow = True

if (M_G["m00"] != 0.0):
    green = True

time_elapsed = time.time() - start_time

if green:
    print(f'Green: {green}')
    starting_dc = (4.5 * 4.0)
    u_tPWM = starting_dc

    car.set_motor(0)
    time_calc = time_elapsed
    time_AD = time_elapsed
    time_loop = time_elapsed
    time_last_calc = time_elapsed

    if (car.adc.read_adc(0)) >=  300:
        light = False
    else: light = True

    dist = car.read_distance()

    while time_elapsed <= 60.0:

        dist = car.read_distance()

        if dist is not None:
            if abs(dist) <= 50:
                break

        time.sleep(args.waitdelay)

        dist = car.read_distance()

        if(time_elapsed - time_loop) > args.loopdelay:
            u_tPWM = PID_controller(rps, time_elapsed, u_tPWM, time_loop, 1.7, 4.0, 0.0, 0.18)
            if u_tPWM > 0 and u_tPWM < 100:
                print(f'New Duty Cycle: {u_tPWM}')
                #pwm_pin.ChangeDutyCycle(u_tPWM)
            elif u_tPWM < 0:
                u_tPWM = 0
                #pwm_pin.ChangeDutyCycle(0)
            elif u_tPWM > 100:
                 u_tPWM = 100
            time_loop = time_elapsed

        car.set_motor(u_tPWM)
        time_elapsed = round(time.time() - start_time,3)

        if (time_elapsed - time_AD) > args.sampledelay:
            b = float(car.adc.read_adc(0))
            time_AD = time_elapsed
            AD_reading.append(b)
            time_vals.append(time_elapsed)

            if len(AD_reading) == 1:
                differences.append(0)
            else: 
                differences.append(AD_reading[-1] - AD_reading[-2])

                if len(differences) > 100:
                    differences.pop(0)

            smoother_val = motor.movingAvg(differences, len(differences)-1, numvals = 10, wrap=0)
            diff_avg.append(smoother_val)
        
            if len(diff_avg) > 400:
                diff_avg.pop(0)

            percentage = 0.20

            if time_elapsed < 1.0:
                positive_threshold = percentage * max(diff_avg)
                negative_threshold = percentage * min(diff_avg)


            val = diff_avg[-1]

            trans_curr = 0

            if (light == True and smoother_val < negative_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = False

            elif (light == False and smoother_val > positive_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = True

            transitions.append(trans_curr)
            RPS.append(rps)


        time_elapsed = round(time.time() - start_time,3)

        if time_elapsed < 0.5:
            transition_times.clear()

        if (time_elapsed - time_calc) > args.calcdelay:
            if len(transition_times) >= 5:
                dt = transition_times[-1] - transition_times[0]
                num_trans = len(transition_times) - 1
                if dt > 0:
                    rps = (num_trans / 5.0) / dt
        
        fname = 'Traffic_Light.txt' # data_PWM_50.txt
        #fname = ('data_kp_.txt')
        with open(fname, 'w') as file:
            for i in range(len(AD_reading)):
                #file.write(f'{time_vals[i]: .4f}\t{AD_reading[i]}\t{RPS[i]: .4f}\n')
                file.write(f'Time: {time_vals[i]}\tAD_Readings: {AD_reading[i]}\tTransitions: {transitions[i]}\tRPS: {RPS[i]}\n')
if yellow:
    print(f'Yellow: {yellow}')

    starting_dc = (4.5 * 4.0)
    u_tPWM = starting_dc

    car.set_motor(0)
    time_calc = time_elapsed
    time_AD = time_elapsed
    time_loop = time_elapsed
    time_last_calc = time_elapsed

    if (car.adc.read_adc(0)) >=  300:
        light = False
    else: light = True

    dist = car.read_distance()

    while time_elapsed <= 60.0:

        dist = car.read_distance()

        if dist is not None:
            if abs(dist) <= 50:
                break

        time.sleep(args.waitdelay)

        dist = car.read_distance()

        if(time_elapsed - time_loop) > args.loopdelay:
            u_tPWM = PID_controller(rps, time_elapsed, u_tPWM, time_loop, 1.7, 4.0, 0.0, 0.18)
            if u_tPWM > 0 and u_tPWM < 100:
                print(f'New Duty Cycle: {u_tPWM}')
                #pwm_pin.ChangeDutyCycle(u_tPWM)
            elif u_tPWM < 0:
                u_tPWM = 0
                #pwm_pin.ChangeDutyCycle(0)
            elif u_tPWM > 100:
                 u_tPWM = 100
            time_loop = time_elapsed

        car.set_motor(u_tPWM)
        time_elapsed = round(time.time() - start_time,3)

        if (time_elapsed - time_AD) > args.sampledelay:
            b = float(car.adc.read_adc(0))
            time_AD = time_elapsed
            AD_reading.append(b)
            time_vals.append(time_elapsed)

            if len(AD_reading) == 1:
                differences.append(0)
            else: 
                differences.append(AD_reading[-1] - AD_reading[-2])

                if len(differences) > 100:
                    differences.pop(0)

            smoother_val = motor.movingAvg(differences, len(differences)-1, numvals = 10, wrap=0)
            diff_avg.append(smoother_val)
        
            if len(diff_avg) > 400:
                diff_avg.pop(0)

            percentage = 0.20

            if time_elapsed < 1.0:
                positive_threshold = percentage * max(diff_avg)
                negative_threshold = percentage * min(diff_avg)


            val = diff_avg[-1]

            trans_curr = 0

            if (light == True and smoother_val < negative_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = False

            elif (light == False and smoother_val > positive_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = True

            transitions.append(trans_curr)
            RPS.append(rps)


        time_elapsed = round(time.time() - start_time,3)

        if time_elapsed < 0.5:
            transition_times.clear()

        if (time_elapsed - time_calc) > args.calcdelay:
            if len(transition_times) >= 5:
                dt = transition_times[-1] - transition_times[0]
                num_trans = len(transition_times) - 1
                if dt > 0:
                    rps = (num_trans / 5.0) / dt
    
   
    starting_dc = (4.5 * 2.0)
    u_tPWM = starting_dc

    while time_elapsed <= 100.0:

        dist = car.read_distance()

        if dist is not None:
            if abs(dist) <= 60:
                car.set_motor(0)
                break

        time.sleep(args.waitdelay)

        dist = car.read_distance()

        if(time_elapsed - time_loop) > args.loopdelay:
            u_tPWM = PID_controller(rps, time_elapsed, u_tPWM, time_loop, 0.2, 2.0, 0.0, 0.04)
            if u_tPWM > 0 and u_tPWM < 100:
                print(f'New Duty Cycle: {u_tPWM}')
                #pwm_pin.ChangeDutyCycle(u_tPWM)
            elif u_tPWM < 0:
                u_tPWM = 0
                #pwm_pin.ChangeDutyCycle(0)
            elif u_tPWM > 100:
                 u_tPWM = 100
            time_loop = time_elapsed

        car.set_motor(u_tPWM)
        time_elapsed = round(time.time() - start_time,3)

        if (time_elapsed - time_AD) > args.sampledelay:
            b = float(car.adc.read_adc(0))
            time_AD = time_elapsed
            AD_reading.append(b)
            time_vals.append(time_elapsed)

            if len(AD_reading) == 1:
                differences.append(0)
            else: 
                differences.append(AD_reading[-1] - AD_reading[-2])

                if len(differences) > 100:
                    differences.pop(0)

            smoother_val = motor.movingAvg(differences, len(differences)-1, numvals = 10, wrap=0)
            diff_avg.append(smoother_val)
        
            if len(diff_avg) > 400:
                diff_avg.pop(0)

            percentage = 0.20

            if time_elapsed < 1.0:
                positive_threshold = percentage * max(diff_avg)
                negative_threshold = percentage * min(diff_avg)


            val = diff_avg[-1]

            trans_curr = 0

            if (light == True and smoother_val < negative_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = False

            elif (light == False and smoother_val > positive_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = True

            transitions.append(trans_curr)
            RPS.append(rps)


        time_elapsed = round(time.time() - start_time,3)

        if time_elapsed < 0.5:
            transition_times.clear()

        if (time_elapsed - time_calc) > args.calcdelay:
            if len(transition_times) >= 5:
                dt = transition_times[-1] - transition_times[0]
                num_trans = len(transition_times) - 1
                if dt > 0:
                    rps = (num_trans / 5.0) / dt
        
    fname = 'Traffic_Light.txt' # data_PWM_50.txt
    #fname = ('data_kp_.txt')
    with open(fname, 'w') as file:
        for i in range(len(AD_reading)):
            #file.write(f'{time_vals[i]: .4f}\t{AD_reading[i]}\t{RPS[i]: .4f}\n')
            file.write(f'Time: {time_vals[i]}\tAD_Readings: {AD_reading[i]}\tTransitions: {transitions[i]}\tRPS: {RPS[i]}\n')
if red:
    print(f'Red: {red}')
    starting_dc = (4.5 * 4.0)
    u_tPWM = starting_dc

    car.set_motor(0)
    time_calc = time_elapsed
    time_AD = time_elapsed
    time_loop = time_elapsed
    time_last_calc = time_elapsed

    if (car.adc.read_adc(0)) >=  300:
        light = False
    else: light = True

    dist = car.read_distance()

    while time_elapsed <= 60.0:

        dist = car.read_distance()

        if dist is not None:
            if abs(dist) <= 300:
                car.set_motor(0)
                break

        time.sleep(args.waitdelay)

        dist = car.read_distance()

        if(time_elapsed - time_loop) > args.loopdelay:
            u_tPWM = PID_controller(rps, time_elapsed, u_tPWM, time_loop, 1.7, 4.0, 0.0, 0.18)
            if u_tPWM > 0 and u_tPWM < 100:
                print(f'New Duty Cycle: {u_tPWM}')
                #pwm_pin.ChangeDutyCycle(u_tPWM)
            elif u_tPWM < 0:
                u_tPWM = 0
                #pwm_pin.ChangeDutyCycle(0)
            elif u_tPWM > 100:
                 u_tPWM = 100
            time_loop = time_elapsed

        car.set_motor(u_tPWM)
        time_elapsed = round(time.time() - start_time,3)

        if (time_elapsed - time_AD) > args.sampledelay:
            b = float(car.adc.read_adc(0))
            time_AD = time_elapsed
            AD_reading.append(b)
            time_vals.append(time_elapsed)

            if len(AD_reading) == 1:
                differences.append(0)
            else: 
                differences.append(AD_reading[-1] - AD_reading[-2])

                if len(differences) > 100:
                    differences.pop(0)

            smoother_val = motor.movingAvg(differences, len(differences)-1, numvals = 10, wrap=0)
            diff_avg.append(smoother_val)
        
            if len(diff_avg) > 400:
                diff_avg.pop(0)

            percentage = 0.20

            if time_elapsed < 1.0:
                positive_threshold = percentage * max(diff_avg)
                negative_threshold = percentage * min(diff_avg)


            val = diff_avg[-1]

            trans_curr = 0

            if (light == True and smoother_val < negative_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = False

            elif (light == False and smoother_val > positive_threshold):
                trans_curr = 1
                transition_times.append(time_elapsed)
                if len(transition_times) > 10:
                    transition_times.pop(0)
                transitionsC = transitionsC + 1
                light = True

            transitions.append(trans_curr)
            RPS.append(rps)


        time_elapsed = round(time.time() - start_time,3)

        if time_elapsed < 0.5:
            transition_times.clear()

        if (time_elapsed - time_calc) > args.calcdelay:
            if len(transition_times) >= 5:
                dt = transition_times[-1] - transition_times[0]
                num_trans = len(transition_times) - 1
                if dt > 0:
                    rps = (num_trans / 5.0) / dt
        
        fname = 'Traffic_Light.txt' # data_PWM_50.txt
        #fname = ('data_kp_.txt')
        with open(fname, 'w') as file:
            for i in range(len(AD_reading)):
                #file.write(f'{time_vals[i]: .4f}\t{AD_reading[i]}\t{RPS[i]: .4f}\n')
                file.write(f'Time: {time_vals[i]}\tAD_Readings: {AD_reading[i]}\tTransitions: {transitions[i]}\tRPS: {RPS[i]}\n')


car.stop()
