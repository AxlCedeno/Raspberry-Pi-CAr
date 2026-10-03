# import module
import time
from picar import PiCar

# test on actual hardware
car = PiCar(mock_car=False, threaded=True)

# see current pin configuration
print(car)

# test ultrasonic
print(f"distance with ultrasonic sensor: {car.read_distance()} cm")
#print(f"distance with ultrasonic sensor: {car.read_distance():.2f} cm")

# test AD
print(f"AD reading of driver side wheel: {car.adc.read_adc(0)}")
print(f"AD reading of passenger side wheel: {car.adc.read_adc(1)}")

# turn on the DC motor- duty_cycle ranges 0-100, forward is optional but is either True (forward) or False (backward)
print("Turn the motor on all the way.")
car.set_motor(100)
time.sleep(2)

# turn DC motor to 50% duty cycle going backwards
print("Turn the motor on half way in reverse.")
car.set_motor(50, forward=False)
time.sleep(2)
car.set_motor(0)

# set the servo positins
# range for servo functions is -10 (down/left) to 10 (up/right) with 0 being center 
print("try various servo positions.")
car.set_nod_servo(-10)
time.sleep(1)
car.set_swivel_servo(5)
time.sleep(1)
car.set_steer_servo(10)
time.sleep(1)

print(f"Accelerometer reading x:  {car.MPU_Read(1):.2f} g")
print(f"Gyroscope reading x:  {car.MPU_Read(4):.2f} degree/s")
print(f"magnetometer reading x:  {car.MPU_Read(7):.2f} uT")

time.sleep(5)

car.stop()
time.sleep(1)
