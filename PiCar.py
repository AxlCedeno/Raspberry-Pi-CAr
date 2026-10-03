import RPi.GPIO as GPIO
import time

import Adafruit_PCA9685 as PWM_HAT
from Adafruit_GPIO.GPIO import RPiGPIOAdapter as Adafruit_GPIO_Adapter
import Adafruit_MCP3008

from picar.CarProcesses import ps_image_stream 
from picar.CarProcesses import ps_ultrasonic_dist
from picar.CarProcesses import ps_keyboard
from picar.ParallelTask import ParallelTask

import pkg_resources
import os.path
import os
import smbus

GPIO.setwarnings(False)    # Ignore warning for now

class PiCar:
    """
    Class to interface with PiCar Hardware 
    """

    # Global Variables
    PICAR_CONFIG_FILE_NAME = "PICAR_CONFIG.txt"

    MOCK_CAR_CONFIG_FILE_NAME = "MOCK_CAR_CONFIG.txt"

    # jdf hack 10/28
    # new car has flipped these
    #SERVO_NOD = 0
    #SERVO_SWIVEL = 1
    #SERVO_STEER = 2
    SERVO_NOD = 2
    SERVO_SWIVEL = 1
    SERVO_STEER = 0

    # Class Variables
    _simulated_hardware = None

    _motor_enable, _motor_pin_1, _motor_pin_2, _motor_pwm = (None, None, None, None)

    motor_state = 0

    _servo_global_pwm = None

    _servo_nod_pin, _servo_nod_pwm = (None, None)
    _servo_nod_left, _servo_nod_middle, _servo_nod_right = (290, 310, 330)

    nod_servo_state = 0

    _servo_swivel_pin, _servo_swivel_pwm = (None, None)
    _servo_swivel_left, _servo_swivel_middle, _servo_swivel_right = (290, 310, 330)

    swivel_servo_state = 0

    _servo_steer_pin, _servo_steer_pwm = (None, None)
    _servo_steer_left, _servo_steer_middle, _servo_steer_right = (290, 310, 330)

    steer_servo_state = 0

    _ultrasonic_trigger, _ultrasonic_echo = (None, None)

    _threaded = None

    _camera_process, _ultrasonic_process, _keyboard_process = (None, None, None)

    adc = None
    
    _bus = smbus.SMBus(1)

    def __init__(self, mock_car=True, pins=None, config_name=None, threaded=False):
        """
        Initialize the PiCar Module
        ---------------------------
        Variables:
        ---------------------------
        mock_car (bool): True for simulated hardware, False to run on actual PiCar
        pins (tuple): List of Custom pins to be used in simulated hardware, in following order:
            (motor_enable, motor_pin_1, motor_pin_2, servo_nod, servo_swivel, servo_steer, ultrasonic_trigger, ultrasonic_echo)
        config_name (String): path to servo configuration file
        threaded (bool): whether the program should run threaded or not
        """

        print("initializing PiCar...")

        if mock_car:
            print("PiCar mode set to MOCK_CAR")
        else:
            print("PiCar mode set to REAL_CAR")

        self._simulated_hardware = mock_car

        GPIO.setmode(GPIO.BOARD)

        print("configuring pins...")

        if pins is not None and mock_car is True:
            # default pins modified, validate custom pins
            if len(pins) != 8:
                raise SystemExit(
                    f"Invalid number of pins supplied: expected 8, found {len(pins)}"
                )
        elif mock_car is True:
            # specify default pins for mock hardware
            pins = (31, 33, 35, 11, 13, 15, 16, 18)
        else:
            # specify default pins for real hardware
            if pins is not None:
                print("Custom pins overridden - not allowed when mock_car is False")
            # jdf hack 10/28
            # pins = (11, 13, 12, 0, 1, 2, 23, 24)
            pins = (41, 43, 42, 2, 1, 0, 15, 16)
        self._init_pins(pins)

        if self._simulated_hardware:
            self._init_mock_car()
        else:
            self._init_car()

        self._threaded = threaded

        if self._threaded:
            print("Program is running in THREADED MODE")
            print(
                "NOTE: All use of the PiCamera module should be through PiCar.get_image"
            )
            print(
                "Any attempt to use the PiCamera module in another context will crash your program"
            )
            self._keyboard_process = ParallelTask(ps_keyboard, ('N') )
            self._camera_process   = ParallelTask(ps_image_stream, ((640, 480), 15))
            self._ultrasonic_process = ParallelTask(
                ps_ultrasonic_dist, (self._ultrasonic_echo, self._ultrasonic_trigger))
            # give the camera a bit to wake up
            time.sleep(1)

        print("initializing software SPI")
        # initialize software SPI
        gpio_adapter = Adafruit_GPIO_Adapter(GPIO, mode=GPIO.BOARD)

        # jdf hack 10/28
        #clk_pin = 40
        #miso_pin = 21
        #mosi_pin = 19
        #cs_pin = 26
        clk_pin = 23
        miso_pin = 21
        mosi_pin = 19
        cs_pin = 24

        self.adc = Adafruit_MCP3008.MCP3008(
            clk=clk_pin, cs=cs_pin, miso=miso_pin, mosi=mosi_pin, gpio=gpio_adapter
        )

        print("looking for servo configuration file...")

        if config_name is not None:
            print("config name overridden")
            self.MOCK_CAR_CONFIG_FILE_NAME = config_name
            self.PICAR_CONFIG_FILE_NAME = config_name

        print(
            f"looking for config file: ./{self.MOCK_CAR_CONFIG_FILE_NAME if mock_car else self.PICAR_CONFIG_FILE_NAME}"
        )

        if os.path.exists(
            self.MOCK_CAR_CONFIG_FILE_NAME if mock_car else self.PICAR_CONFIG_FILE_NAME
        ):
            print("servo configuration found!")
            with open(
                self.MOCK_CAR_CONFIG_FILE_NAME
                if mock_car
                else self.PICAR_CONFIG_FILE_NAME,
                "r",
            ) as config:
                configuration = config.readlines()
                # 9 elements + newline at the end
                if len(configuration) < 9:
                    print(
                        f"Invalid configuration file, expected 9 elements, found {len(configuration)}"
                    )
                else:
                    self.configure_nod_servo_positions(
                        int(configuration[1]),
                        int(configuration[0]),
                        int(configuration[2]),
                    )
                    self.configure_swivel_servo_positions(
                        int(configuration[4]),
                        int(configuration[3]),
                        int(configuration[5]),
                    )
                    self.configure_steer_servo_positions(
                        int(configuration[7]),
                        int(configuration[6]),
                        int(configuration[8]),
                    )
        else:
            print("servo configuration not found, using default values")

        print("PiCar configured successfully")

        print(self)

        print("setting motors to default positions...")

        # set initial component states
        self.set_motor(0)

        self.set_nod_servo(0)
        self.set_swivel_servo(0)
        self.set_steer_servo(0)

    def _init_pins(self, pins):
        """
        pins: List of pins to be configured for hardware, in following order:
        (motor_pin_1, motor_pin_2, motor_enable, servo_nod, servo_swivel, servo_steer)
        """

        (
            self._motor_enable,
            self._motor_pin_1,
            self._motor_pin_2,
            self._servo_nod_pin,
            self._servo_swivel_pin,
            self._servo_steer_pin,
            self._ultrasonic_trigger,
            self._ultrasonic_echo,
        ) = pins

        # jdf hack 10/28
        # setup motor pins
        #GPIO.setup(self._motor_enable, GPIO.OUT, initial=GPIO.LOW)
        #GPIO.setup(self._motor_pin_1, GPIO.OUT, initial=GPIO.LOW)
        #GPIO.setup(self._motor_pin_2, GPIO.OUT, initial=GPIO.LOW)

        # setup servo pins
        #if self._simulated_hardware:
        #    GPIO.setup(self._servo_nod_pin, GPIO.OUT)
        #    GPIO.setup(self._servo_swivel_pin, GPIO.OUT)
        #    GPIO.setup(self._servo_steer_pin, GPIO.OUT)
        # setup motor pins

        # setup servo pins and motor pins only on simulated hardware
        if self._simulated_hardware:
            GPIO.setup(self._servo_nod_pin, GPIO.OUT)
            GPIO.setup(self._servo_swivel_pin, GPIO.OUT)
            GPIO.setup(self._servo_steer_pin, GPIO.OUT)
            GPIO.setup(self._motor_enable, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self._motor_pin_1, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self._motor_pin_2, GPIO.OUT, initial=GPIO.LOW)

        # setup ultrasonic pins
        GPIO.setup(self._ultrasonic_trigger, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._ultrasonic_echo, GPIO.IN)

    def _init_mock_car(self):
        """
        Initialize car variables for use with RPi Treated as Mock Car
        """

        self._servo_nod_pwm = GPIO.PWM(self._servo_nod_pin, 50)
        self._servo_swivel_pwm = GPIO.PWM(self._servo_swivel_pin, 50)
        self._servo_steer_pwm = GPIO.PWM(self._servo_steer_pin, 50)

        self.configure_nod_servo_positions(5, 7.5, 10)
        self.configure_swivel_servo_positions(5, 7.5, 10)
        self.configure_steer_servo_positions(5, 7.5, 10)

        self._servo_nod_pwm.start(self._servo_nod_middle)
        self._servo_swivel_pwm.start(self._servo_swivel_middle)
        self._servo_steer_pwm.start(self._servo_steer_middle)

        self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        self._motor_pwm.start(0)
        pass

    def _init_car(self):
        """
        Initialize car variables for use with the actual Adeept Car
        """
        self._servo_global_pwm = PWM_HAT.PCA9685()
        # jdf hack 10/28
        #self._servo_global_pwm.set_pwm_freq(60)
        self._servo_global_pwm.set_pwm_freq(50)

        #self._motor_pwm = GPIO.PWM(self._motor_enable, 1000)
        #self._motor_pwm.start(0)
        self._motor_pwm = self._servo_global_pwm
        
        # intialize the IMU20948
        self._MPU_Init()

        pass

    def set_motor(self, duty_cycle, forward=True):
        # jdf hack 10/28
        #"""
        #forward (bool):direction of motor spin
        #duty_cycle (int): 0->100
        #"""
        #duty_cycle = duty_cycle if duty_cycle < 100 else 100
        #duty_cycle = duty_cycle if duty_cycle >= 0 else 0
        #GPIO.output(self._motor_pin_1, GPIO.LOW if forward else GPIO.HIGH)
        #GPIO.output(self._motor_pin_2, GPIO.HIGH if forward else GPIO.LOW)
        #self._motor_pwm.ChangeDutyCycle(duty_cycle)
        #self.motor_state = duty_cycle
        """
        Uses pwd channels 12, 13 for one side and 14, 15 on the other
        14, 15 will turn the opposite way due to mounting on other side
        """
        duty = int(duty_cycle * 4095/100)
        if (duty < 0) :
          duty = 0
        elif (duty > 4095) :
          duty = 4095
        if (forward) :
            self._motor_pwm.set_pwm(12, 0, duty)
            self._motor_pwm.set_pwm(14, 0, duty)
            self._motor_pwm.set_pwm(13, 0, 0)
            self._motor_pwm.set_pwm(15, 0, 0)
        else :
            self._motor_pwm.set_pwm(12, 0, 0)
            self._motor_pwm.set_pwm(14, 0, 0)
            self._motor_pwm.set_pwm(13, 0, duty)
            self._motor_pwm.set_pwm(15, 0, duty)

    def reset(self):
        """
        Reset the hardware - will set all motors and servos to neutral positions
        """
        self.set_motor(0)

        self.set_nod_servo(0)
        self.set_steer_servo(0)
        self.set_swivel_servo(0)

    def stop(self):
        """
        Stop the hardware - will kill PWM instance for all motors and servos, and cleanup GPIO
        """
        
        # jdf hack 10/28
        #self._motor_pwm.stop()
        #if self._simulated_hardware:
        #    self._servo_nod_pwm.stop()
        #    self._servo_swivel_pwm.stop()
        #    self._servo_steer_pwm.stop()
        if self._simulated_hardware:
            self._servo_nod_pwm.stop()
            self._servo_swivel_pwm.stop()
            self._servo_steer_pwm.stop()
            self._motor_pwm.stop()
        else :
            self._servo_global_pwm.set_pwm(2, 0, 0)
            self._servo_global_pwm.set_pwm(1, 0, 0)
            self._servo_global_pwm.set_pwm(0, 0, 0)
            self._motor_pwm.set_pwm(12, 0, 0)
            self._motor_pwm.set_pwm(13, 0, 0)
            self._motor_pwm.set_pwm(14, 0, 0)
            self._motor_pwm.set_pwm(15, 0, 0)

        GPIO.cleanup()

        if self._threaded:      # clean up the threads still active
            self._ultrasonic_process.stop_thread()
            self._camera_process.stop_thread()
            self._keyboard_process.stop_thread()
            os.system('reset')


    def _calc_servo_duty_cycle(self, left, middle, right, amount, is_left):
        return (
            middle - (middle - left) * amount / 10
            if is_left
            else (right - middle) * amount / 10 + middle
        )

    def _set_servo(self, servo, value=None, raw=False):
        """
        Generic set servo code
        Wrapped by specific set servo commands so as to not change the API for students
        TODO: change API to use this for simplicity
        """
        if servo not in [PiCar.SERVO_NOD, PiCar.SERVO_SWIVEL, PiCar.SERVO_STEER]:
            raise SystemExit(
                f"Invalid servo specified to set_servo. Expected one of PiCar.SERVO_NOD, PiCar.SERVO_SWIVEL, PiCar.SERVO_STEER"
            )
        # handle special input cases
        if raw:
            if servo == PiCar.SERVO_NOD:
                self._servo_global_pwm.set_pwm(self._servo_nod_pin, 0, raw)
                self.nod_servo_state = raw
            elif servo == PiCar.SERVO_SWIVEL:
                self._servo_global_pwm.set_pwm(self._servo_swivel_pin, 0, raw)
                self.nod_swivel_state = raw
            elif servo == PiCar.SERVO_STEER:
                self._servo_global_pwm.set_pwm(self._servo_steer_pin, 0, raw)
                self.nod_steer_state = raw
            return
        if not (isinstance(value, int) or isinstance(value, float)):
            raise SystemExit(
                f"value argument must be numeric between -10 and 10. Expected int or float, found {type(value)}"
            )

        # cast value to correct range
        safe_value = max(min(10, value), -10)
        if safe_value != value:
            print(
                f"WARNING: value passed to set_[nod|swivel|steer]_servo exceeds expected range. Expected -10 <= value <= 10, found {value}"
            )
            print(f"Casting value to {safe_value}")
        value = safe_value

        is_left, amount = (value < 0, abs(value))
        duty_cycle = None
        if servo == PiCar.SERVO_NOD:
            duty_cycle = self._calc_servo_duty_cycle(
                self._servo_nod_left,
                self._servo_nod_middle,
                self._servo_nod_right,
                amount,
                is_left,
            )
        elif servo == PiCar.SERVO_SWIVEL:
            duty_cycle = self._calc_servo_duty_cycle(
                self._servo_swivel_left,
                self._servo_swivel_middle,
                self._servo_swivel_right,
                amount,
                is_left,
            )
        elif servo == PiCar.SERVO_STEER:
            duty_cycle = self._calc_servo_duty_cycle(
                self._servo_steer_left,
                self._servo_steer_middle,
                self._servo_steer_right,
                amount,
                is_left,
            )

        if self._simulated_hardware:
            if servo == PiCar.SERVO_NOD:
                self._servo_nod_pwm.ChangeDutyCycle(duty_cycle)
            elif servo == PiCar.SERVO_SWIVEL:
                self._servo_swivel_pwm.ChangeDutyCycle(duty_cycle)
            elif servo == PiCar.SERVO_STEER:
                self._servo_steer_pwm.ChangeDutyCycle(duty_cycle)
        else:
            if servo == PiCar.SERVO_NOD:
                self._servo_global_pwm.set_pwm(self._servo_nod_pin, 0, int(duty_cycle))
            elif servo == PiCar.SERVO_SWIVEL:
                self._servo_global_pwm.set_pwm(
                    self._servo_swivel_pin, 0, int(duty_cycle)
                )
            elif servo == PiCar.SERVO_STEER:
                self._servo_global_pwm.set_pwm(
                    self._servo_steer_pin, 0, int(duty_cycle)
                )
        if servo == PiCar.SERVO_NOD:
            self.nod_servo_state = value
        elif servo == PiCar.SERVO_SWIVEL:
            self.swivel_servo_state = value
        elif servo == PiCar.SERVO_STEER:
            self.steer_servo_state = value

    def configure_nod_servo_positions(self, left=None, middle=None, right=None):
        """
        Provide alternate nod servo positions
        Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
        left (int): servo duty cycle for left position
        middle (int): servo duty cycle for middle position
        right (int): servo duty cycle for right position
        """
        if left is None:
            left = self._servo_nod_left
        if middle is None:
            middle = self._servo_nod_middle
        if right is None:
            right = self._servo_nod_right
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (left, middle, right)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        print(f"nod : {left} , {middle} , {right}")
        self._servo_nod_left = left
        self._servo_nod_middle = middle
        self._servo_nod_right = right

    def set_nod_servo(self, value, raw=False):
        """
        value (int): between -10 and 10, -10 being max down, 0 being center, and 10 being max up
        raw (int): only to be used for TA debugging
        """
        self._set_servo(PiCar.SERVO_NOD, value, raw)

    def configure_swivel_servo_positions(self, left=None, middle=None, right=None):
        """
        Provide alternate swivel servo positions
        Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
        left (int): servo duty cycle for left position
        middle (int): servo duty cycle for middle position
        right (int): servo duty cycle for right position
        """
        if left is None:
            left = self._servo_swivel_left
        if middle is None:
            middle = self._servo_swivel_middle
        if right is None:
            right = self._servo_swivel_right
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (left, middle, right)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )
        print(f"swivel : {left} , {middle} , {right}")
        self._servo_swivel_left = left
        self._servo_swivel_middle = middle
        self._servo_swivel_right = right

    def set_swivel_servo(self, value, raw=False):
        """
        value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
        raw (int): only to be used for TA debugging
        """
        self._set_servo(PiCar.SERVO_SWIVEL, value, raw)

    def configure_steer_servo_positions(self, left=None, middle=None, right=None):
        """
        Provide alternate steer servo positions
        Note: when setting servo positon, values between left and middle or left and right are linearly interpolated
        left (int): servo duty cycle for left position
        middle (int): servo duty cycle for middle position
        right (int): servo duty cycle for right position
        """
        if left is None:
            left = self._servo_steer_left
        if middle is None:
            middle = self._servo_steer_middle
        if right is None:
            right = self._servo_steer_right
        if False in [
            isinstance(x, int) or isinstance(x, float) for x in (left, middle, right)
        ]:
            raise SystemExit(
                f"All args must be integer values, expected int, int, int, found: {type(left)}, {type(middle)}, {type(right)}"
            )

        print(f"steer : {left} , {middle} , {right}")
        self._servo_steer_left = left
        self._servo_steer_middle = middle
        self._servo_steer_right = right

    def set_steer_servo(self, value, raw=False):
        """
        Set the steer servo 
        value (int): between -10 and 10, -10 being max left, 0 being center, and 10 being max right
        raw (int): only to be used for TA debugging 
        """
        self._set_servo(PiCar.SERVO_STEER, value, raw)

    def read_distance(self):
        """
        Read ultrasonic sensor
        return (double): distance in cm from object as detected by ultrasonic sensor
        """
        if self._threaded:
            return self._ultrasonic_process.get_result()[0]
        else:
            # activate trigger
            GPIO.output(self._ultrasonic_trigger, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self._ultrasonic_trigger, GPIO.LOW)

            # get send and recieve time
            while not GPIO.input(self._ultrasonic_echo):
                pass
            echo_start = time.time()
            while GPIO.input(self._ultrasonic_echo):
                pass
            echo_end = time.time()
            # compute one way distance in cm from a two way time in seconds
            return (echo_end - echo_start) * 340 * 100 / 2

    def get_image(self):
        """
        returns an image object
        NOTE: only valid if the program is threaded
        """
        if not self._threaded:
            raise SystemExit(
                "FATAL: get_image can only be called when PiCar is run in threaded mode"
            )

        return self._camera_process.get_result()[0]

    def get_keyin(self):
        """
        returns an keystroke as a char
        NOTE: only valid if the program is threaded
        """
        if not self._threaded:
            raise SystemExit(
                "FATAL: get_keyin can only be called when PiCar is run in threaded mode"
            )

        return self._keyboard_process.get_result()[0]

    def __repr__(self):
        """
        Format PiCar for print representation
        print(PiCar) will show currently configured pins
        TODO: Add additional variables (i.e. simulated or not, whether all PWMs are configured, etc)
        TODO fix version
        TODO: Add config filename
        with open("./VERSION", "r") as ver:
            version = ver.read().strip()
        print("ok")
        """
        entries = [
            ["State"],
            ["Motor", self.motor_state],
            ["Nod Servo", self.nod_servo_state],
            ["Swivel Servo", self.swivel_servo_state],
            ["Steer Servo", self.steer_servo_state],
            ["Configuration"],
            [
                "Nod Servo (l, m, r)",
                self._servo_nod_left,
                self._servo_nod_middle,
                self._servo_nod_right,
            ],
            [
                "Swivel Servo (l, m, r)",
                self._servo_swivel_left,
                self._servo_swivel_middle,
                self._servo_swivel_right,
            ],
            [
                "Steer Servo (l, m, r)",
                self._servo_steer_left,
                self._servo_steer_middle,
                self._servo_steer_right,
            ],
            ["Pins"],
            ["Motor Enable", self._motor_enable],
            ["Motor 1", self._motor_pin_1],
            ["Motor 2", self._motor_pin_2],
            ["Nod Servo", self._servo_nod_pin],
            ["Swivel Servo", self._servo_swivel_pin],
            ["Steer Servo", self._servo_steer_pin],
            ["Trigger", self._ultrasonic_trigger],
            ["Echo", self._ultrasonic_echo],
        ]

        rep = f"PiCar Version 0.4.15:\n"

        col_sizes = compute_column_lengths(entries)

        for entry in entries:
            if len(entry) == 1:
                rep += "-" * sum(col_sizes)
                rep += "\n"
                rep += f"{entry[0]}\n"
                rep += "-" * sum(col_sizes)
                rep += "\n"
            else:
                for index in range(0, len(entry)):
                    rep += str(entry[index])
                    rep += (col_sizes[index] - len(str(entry[index]))) * " "
                rep += "\n"

        rep += "-" * sum(col_sizes)
        """
        rep += "---------------------\n"
        rep += "Motor:\n"
        rep += f"Enable Pin: {self._motor_enable} Pin 1: {self._motor_pin_1} Pin 2:{self._motor_pin_2}\n"
        rep += "---------------------\n"
        rep += "Servos:\n"
        rep += f"Nod Pin: {self._servo_nod_pin} Swivel Pin: {self._servo_swivel_pin} Steer Pin: {self._servo_steer_pin}\n"
        rep += "---------------------\n"
        rep += "Ultrasonic:\n"
        rep += f"Trigger Pin: {self._ultrasonic_trigger} Echo Pin: {self._ultrasonic_echo}\n"
        rep += "=====================\n"
        """

        return rep
    
   # MPU ICM20948
    def _bank_select(self, bank):
    #Switch ICM20948 register bank (0–3)
        REG_BANK_SEL = 0x7F
        self._bus.write_byte_data(self._device_address, REG_BANK_SEL, bank << 4)

    def _MPU_Init(self):
        self._device_address = 0x68   # 0x69 if AD0 pin is HIGH

        # ------------------------------------------------------------------ #
        #  Bank 0 — Power on and bypass setup                                 #
        # ------------------------------------------------------------------ #
        self._bank_select(0)

        PWR_MGMT_1  = 0x06
        PWR_MGMT_2  = 0x07
        INT_PIN_CFG = 0x0F
        INT_ENABLE  = 0x10
        I2C_MST_EN  = 0x03   # bit in USER_CTRL to disable I2C master

        USER_CTRL   = 0x03

        # Step 1 — Wake device, auto-select clock (clears SLEEP bit)
        self._bus.write_byte_data(self._device_address, PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # Step 2 — Enable all accel + gyro axes
        self._bus.write_byte_data(self._device_address, PWR_MGMT_2, 0x00)
        time.sleep(0.05)

        # Step 3 — CRITICAL: disable I2C master BEFORE enabling bypass
        # If I2C master is active it "owns" the aux bus and bypass won't work
        self._bus.write_byte_data(self._device_address, USER_CTRL, 0x00)  # I2C_MST_EN = 0
        time.sleep(0.05)

        # Step 4 — Enable bypass mode so AK09916 is visible on the main I2C bus
        self._bus.write_byte_data(self._device_address, INT_PIN_CFG, 0x02)  # BYPASS_EN = 1
        time.sleep(0.1)   # longer delay — give the bus time to switch over

        # Step 5 — Enable data-ready interrupt
        self._bus.write_byte_data(self._device_address, INT_ENABLE, 0x01)

        # ------------------------------------------------------------------ #
        #  Verify AK09916 is reachable before configuring it                  #
        # ------------------------------------------------------------------ #
        AK09916_ADDR = 0x0C
        AK09916_WIA2 = 0x01   # "Who I Am" register — should return 0x09

        try:
            who = self._bus.read_byte_data(AK09916_ADDR, AK09916_WIA2)
            if who != 0x09:
                raise RuntimeError(f"AK09916 WIA2 returned 0x{who:02X}, expected 0x09")
            print(f"AK09916 detected OK (WIA2=0x{who:02X})")
        except OSError as e:
            raise RuntimeError(
                "AK09916 not found on I2C bus. "
                "Check bypass wiring or confirm AD0 pin level (try address 0x69)."
            ) from e

        # ------------------------------------------------------------------ #
        #  Configure AK09916                                                   #
        # ------------------------------------------------------------------ #
        AK09916_CNTL3           = 0x32   # soft reset
        AK09916_CNTL2           = 0x31   # operating mode
        AK09916_CONT_MODE_100HZ = 0x08   # continuous mode 4, 100 Hz

        # Soft-reset magnetometer
        self._bus.write_byte_data(AK09916_ADDR, AK09916_CNTL3, 0x01)
        time.sleep(0.1)

        # Set continuous measurement at 100 Hz
        self._bus.write_byte_data(AK09916_ADDR, AK09916_CNTL2, AK09916_CONT_MODE_100HZ)
        time.sleep(0.05)

        # ------------------------------------------------------------------ #
        #  Bank 2 — Gyroscope and Accelerometer configuration                 #
        # ------------------------------------------------------------------ #
        self._bank_select(2)

        GYRO_SMPLRT_DIV  = 0x00
        GYRO_CONFIG_1    = 0x01
        ACCEL_SMPLRT_DIV = 0x10
        ACCEL_CONFIG     = 0x14

        # Gyro sample rate ≈ 100 Hz
        self._bus.write_byte_data(self._device_address, GYRO_SMPLRT_DIV, 0x0A)
        # Gyro: ±2000 °/s + DLPF enabled
        self._bus.write_byte_data(self._device_address, GYRO_CONFIG_1, 0x19)

        # Accel sample rate ≈ 100 Hz
        self._bus.write_byte_data(self._device_address, ACCEL_SMPLRT_DIV,     0x00)
        self._bus.write_byte_data(self._device_address, ACCEL_SMPLRT_DIV + 1, 0x0A)
        # Accel: ±2g + DLPF enabled
        self._bus.write_byte_data(self._device_address, ACCEL_CONFIG, 0x01)

        # Return to Bank 0 as default
        self._bank_select(0)

    def MPU_Read(self, val2read):
        """
        Read a single sensor axis from ICM20948 or AK09916.

        val2read:
            1 – accel X  (g)
            2 – accel Y  (g)
            3 – accel Z  (g)
            4 – gyro  X  (°/s)
            5 – gyro  Y  (°/s)
            6 – gyro  Z  (°/s)
            7 – mag   X  (µT)
            8 – mag   Y  (µT)
            9 – mag   Z  (µT)
        """
        if not (1 <= val2read <= 9):
            raise ValueError(f"Invalid val2read: {val2read}. Must be 1–9.")

        # ------------------------------------------------------------------ #
        #  Accelerometer / Gyroscope  (ICM20948, Bank 0, big-endian)          #
        # ------------------------------------------------------------------ #
        if val2read <= 6:
            ACCEL_XOUT_H = 0x2D
            GYRO_XOUT_H  = 0x33

            self._bank_select(0)

            if   val2read == 1: addr = ACCEL_XOUT_H
            elif val2read == 2: addr = ACCEL_XOUT_H + 2
            elif val2read == 3: addr = ACCEL_XOUT_H + 4
            elif val2read == 4: addr = GYRO_XOUT_H
            elif val2read == 5: addr = GYRO_XOUT_H + 2
            elif val2read == 6: addr = GYRO_XOUT_H + 4

            # Big-endian: high byte first
            high  = self._bus.read_byte_data(self._device_address, addr)
            low   = self._bus.read_byte_data(self._device_address, addr + 1)
            value = (high << 8) | low

            # Two's complement → signed
            if value > 32767:
                value -= 65536

            # Scale using configured full-scale ranges:
            #   Accel ±2g      → 16384 LSB/g
            #   Gyro  ±2000°/s →    16.4 LSB/(°/s)
            if val2read <= 3:
                return value / 16384.0   # g
            else:
                return value / 16.4      # °/s

        # ------------------------------------------------------------------ #
        #  Magnetometer  (AK09916, address 0x0C, little-endian)               #
        # ------------------------------------------------------------------ #
        else:
            AK09916_ADDR = 0x0C
            HXL = 0x11   # Mag X low byte
            HYL = 0x13   # Mag Y low byte
            HZL = 0x15   # Mag Z low byte
            ST2 = 0x18   # Status 2 — must be read after data to unlatch next sample

            if   val2read == 7: addr = HXL
            elif val2read == 8: addr = HYL
            elif val2read == 9: addr = HZL

            # Little-endian: low byte first
            low   = self._bus.read_byte_data(AK09916_ADDR, addr)
            high  = self._bus.read_byte_data(AK09916_ADDR, addr + 1)
            value = (high << 8) | low

            # Two's complement → signed
            if value > 32767:
                value -= 65536

            # Read ST2 to release the data latch — required by AK09916 datasheet
            self._bus.read_byte_data(AK09916_ADDR, ST2)

            # Fixed sensitivity: 0.15 µT/LSB (AK09916 has no configurable range)
            return value * 0.15   # µT
            
def compute_column_lengths(data):
    lengths = []
    index = 0
    has_entity = True
    while has_entity:
        has_entity = False
        max_len = 0
        for item in data:
            if len(item) > index:
                has_entity = True
                if len(str(item[index])) > max_len:
                    max_len = len(str(item[index]))
        max_len += 4  # pad each column by 4 spaces
        max_len = max_len + 4 - (max_len % 4)  # round to nearest tab
        lengths.append(max_len)
        index += 1

    return lengths
