from picar import PiCar
import time
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true')
parser.add_argument('--tim', type=float, default=10.0)
args = parser.parse_args()

car = PiCar(mock_car=args.mock_car, threaded=True)

car.set_motor(20)

car.set_nod_servo(0)
time.sleep(1)
car.set_swivel_servo(0)
time.sleep(1)
steer_pos = 0
car.set_steer_servo(0)

start_time = time.time()
elapsed = round(time.time() - start_time,3)

print("'a' = left, 'd' = right")
print()

while (elapsed < args.tim):
# read ultrasonic distance
    dist = car.read_distance()
    key = car.get_keyin()

    #if dist is not None:
    #    print(f'distance: {dist:.2f} cm')
    elapsed = round(time.time() - start_time,3)
    
    if dist is not None:
        if (dist <= 100):
            break

    if key is not None:
        # left
        if key == 'a':
            steer_pos += 0.5
        # right
        elif key == 'd':
            steer_pos -= 0.5
        
        if key == 's':
            steer_pos = 0

        if (steer_pos < 10 and steer_pos > -10):
            car.set_steer_servo(steer_pos)
        elif (steer_pos >= 10):
            car.set_steer_servo(10)
        elif (steer_pos <= -10):
            car.set_steer_servo(-10)
    time.sleep(0.1)

car.stop()
