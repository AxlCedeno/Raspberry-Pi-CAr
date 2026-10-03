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
parser.add_argument('--tim', type=int, default=10)
parser.add_argument("--sampledelay", type=float, default=0.005,help="Delay between samples (seconds)")
parser.add_argument("--calcdelay",type = float, default=0.10, help = "Delay between calculating the motor speed")
parser.add_argument("--loopdelay",type = float, default=1.0, help = "Delay between Starting the Photoresistor Loop")
parser.add_argument("--waitdelay",type = float, default=0.0, help = "Delay before Starting the Motor")
parser.add_argument("--Drps", type=float, default= 4.0, help="Desired RPS")
parser.add_argument("--kp", type = float, default = 0.0, help ="Proportional Control")
parser.add_argument("--ki", type = float, default = 0.0, help = "Integral Control")
parser.add_argument("--kd", type = float, default = 0.0, help ="Derivative Control")
parser.add_argument('--debug', action='store_true')
args = parser.parse_args()

# init car
car = PiCar(mock_car=args.mock_car)

if args.debug:
    print(car)

def PID_controller(rps, time_elapsed, last_pwm, last_time):
    error = args.Drps - rps
    Error.append(error)
    errorT = sum(Error)
    P = args.kp * error
    I = args.ki * (errorT) * (time_elapsed - last_time)
    if len(Error) > 1:
        D = args.kd * ((Error[-1] - Error[-2])/(time_elapsed - last_time))
    else: D = 0

    u_t = P + I + D
    u_tPWM = last_pwm + (4.5 * u_t)

    return u_tPWM

start_time = time.time()
time_elapsed = round(time.time() - start_time,3)

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

starting_dc = (4.5 * args.Drps)
u_tPWM = starting_dc

car.set_motor(0)
time_calc = time_elapsed
time_AD = time_elapsed
time_loop = time_elapsed
time_last_calc = time_elapsed

if (car.adc.read_adc(0)) >=  300:
    light = False
else: light = True

while (time_elapsed) < args.tim:
    time.sleep(args.waitdelay)

    if(time_elapsed - time_loop) > args.loopdelay:
        u_tPWM = PID_controller(rps, time_elapsed, u_tPWM, time_loop)
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
car.stop()

fname = 'data_10_test.txt' # data_PWM_50.txt
#fname = ('data_kp_.txt')
with open(fname, 'w') as file:
    for i in range(len(AD_reading)):
        #file.write(f'{time_vals[i]}\t{AD_reading[i]}\t{RPS[i]}\n')
        file.write(f'Time: {time_vals[i]}\tAD_Readings: {AD_reading[i]}\tTransitions: {transitions[i]}\tRPS: {RPS[i]}\n')

#fname = input('Enter filename: ')
file = open(fname, "r")
data = file.read().splitlines() # split lines into an array
MAXSIZE = len(data)

#pltname = fname.removeprefix('data_PWM_')
#pltname = fname.removesuffix('.txt')

time_vals = [0]*MAXSIZE
AD_reading = [0]*MAXSIZE
Transitions = [0] *MAXSIZE
RPS = [0]*MAXSIZE
fft_vals = [0]*MAXSIZE
sse_calc = []

Light = False
i=0
d=0
e=0
f=0
for dat in data:
   values = dat.split() # split on white space
   time_vals[i] = float(values[1]) # first item in file is time
   AD_reading[i] = float(values[3]) # second is the value
   
   if (AD_reading[i] > 410 and i == 0):
    Light = False
   elif(AD_reading[i] <= 410 and i == 0): Light = True

   if (float(values[5]) == 1 and Light):
    Transitions[i] = -1
    Light = False
   elif (float(values[5]) == 1):
    Transitions[i] = 1
    Light = True
   else: Transitions[i] = float(values[5])
   RPS[i] = float(values[7])
   if (RPS[i] >= 0.7 * args.Drps and d == 0):
    rise_time = time_vals[i]
    d = 1
   if ((RPS[i] + RPS[i-1]) < 0.05 or e != 0):
    sse_calc.append(RPS[i])
    e=e+1
   if (f == 0):
    overshoot = max(RPS)
   if (time_vals[i] >= 3.5):
    f == 1
   i = i + 1

#dt = np.mean(np.diff(time_vals))
#ADC_array = np.array(ADC_reading)

#N = len(RPS_array)
#fft_values = fft(ADC_array)
#frequencies = np.linspace(0, dt/2, N/2)
#fft_magnitude = (2.0 / N) * np.abs(fft_values[:N // 2])
#frequencies = frequencies[:N // 2]

#xmarks = np.linspace(time_vals[0], time_vals[MAXSIZE-1], 5)
#plt.xticks(np.linspace(frequencies, frequencies[-1], 6))

plt.figure()
plt.plot(time_vals, RPS)
#plt.plot(frequencies, fft_magnitude)
plt.grid(True)
plt.xlabel('Time')
plt.ylabel('RPS')

plt.savefig('rps_10_test.png')

over_shoot = overshoot - args.Drps
if (over_shoot < 0):
    over_shoot = 0.000

sse = args.Drps - (sum(sse_calc)/len(sse_calc))

print(f'Rise Time: {rise_time}')
print(f'Overshoot: {over_shoot}')
print(f'Steady State Error: {sse}')
