# import module
from picar import PiCar, configure
import time
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', 
                    help='If not present, run on car, otherwise mock hardware')
args = parser.parse_args()

# test on actual hardware
car = PiCar(mock_car=args.mock_car)

for i in range(3):
   setting = (i-1)*10
   car.set_swivel_servo(setting)
   print(f'car swivel: {car.swivel_servo_state}')
   time.sleep(1)
car.set_swivel_servo(0)
time.sleep(1)

for i in range(3):
   setting = (i-1)*10
   car.set_steer_servo(setting)
   print(f'car steer: {car.steer_servo_state}')
   time.sleep(1)
car.set_steer_servo(0)
time.sleep(1)

for i in range(3):
   setting = (i-1)*10
   car.set_nod_servo(setting)
   print(f'car nod: {car.nod_servo_state}')
   time.sleep(1)
car.set_nod_servo(0)
time.sleep(1)
