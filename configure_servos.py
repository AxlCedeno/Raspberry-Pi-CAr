# import module
from picar import PiCar, configure
import time
import argparse

parser = argparse.ArgumentParser(description='Data for this program.')
parser.add_argument('--mock_car', action='store_true', 
                    help='If not present, run on car, otherwise mock hardware')
args = parser.parse_args()

# select mock car or on actual PiCar
car = PiCar(mock_car=args.mock_car)

print(f'Start configuring servo-motors!')

# configure each servo-motor on PiCar or single servomotor on mock car
configure.configure_car(car)
