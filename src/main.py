import vex
# from vex import *


def map_range(
    x: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """
    Maps a number from one range to another. Somewhat similar to the Arduino
    :attr:`map()` function, but returns a floating point result, and
    constrains the output value to be between :attr:`out_min` and
    :attr:`out_max`. If :attr:`in_min` is greater than :attr:`in_max` or
    :attr:`out_min` is greater than :attr:`out_max`, the corresponding range
    is reversed, allowing, for example, mapping a range of 0-10 to 50-0.

    See also :py:func:`map_unconstrained_range`

    .. code-block::

        from adafruit_simplemath import map_range

        percent = 23
        screen_width = 320  # or board.DISPLAY.width
        x = map_range(percent, 0, 100, 0, screen_width - 1)
        print("X position", percent, "% from the left of screen is", x)

    :param float x: Value to convert
    :param float in_min: Start value of input range.
    :param float in_max: End value of input range.
    :param float out_min: Start value of output range.
    :param float out_max: End value of output range.
    :return: Returns value mapped to new range.
    :rtype: float
    """

    mapped = map_unconstrained_range(x, in_min, in_max, out_min, out_max)
    return constrain(mapped, out_min, out_max)

boc1 = "(me)"

def map_unconstrained_range(
    x: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """
    Maps a number from one range to another. Somewhat similar to the Arduino
    :attr:`map()` function, but returns a floating point result, and
    does not constrain the output value to be between :attr:`out_min` and
    :attr:`out_max`. If :attr:`in_min` is greater than :attr:`in_max` or
    :attr:`out_min` is greater than :attr:`out_max`, the corresponding range
    is reversed, allowing, for example, mapping a range of 0-10 to 50-0.

    See also :py:func:`map_range`

    .. code-block::

        from adafruit_simplemath import map_unconstrained_range

        celsius = -20
        fahrenheit = map_unconstrained_range(celsius, 0, 100, 32, 212)
        print(celsius, "degress Celsius =", fahrenheit, "degrees Fahrenheit")

    :param float x: Value to convert
    :param float in_min: Start value of input range.
    :param float in_max: End value of input range.
    :param float out_min: Start value of output range.
    :param float out_max: End value of output range.
    :return: Returns value mapped to new range.
    :rtype: float
    """
    in_range = in_max - in_min
    in_delta = x - in_min
    if in_range != 0:
        mapped = in_delta / in_range
    elif in_delta != 0:
        mapped = in_delta
    else:
        mapped = 0.5
    mapped *= out_max - out_min
    mapped += out_min

    return mapped

def get_direction_and_speed(speed):
    direction = vex.DirectionType.FORWARD
    if speed < 0:
        direction = vex.DirectionType.REVERSE
        speed *= -1
    speed = map_range(speed, 0, 1.0, 0, 100)
    return direction, speed


def constrain(x: float, out_min: float, out_max: float) -> float:
    """Constrains :attr:`x` to be within the inclusive range
    [:attr:`out_min`, :attr:`out_max`]. Sometimes called :attr:`clip` or
    :attr:`clamp` in other libraries. :attr:`out_min` should be less than or
    equal to :attr:`out_max`.
    If :attr:`x` is less than :attr:`out_min`, return :attr:`out_min`.
    If :attr:`x` is greater than :attr:`out_max`, return :attr:`out_max`.
    Otherwise just return :attr:`x`.
    If :attr:`max_value` is less than :attr:`min_value`, they will be swapped.

    :param float x: Value to constrain
    :param float out_min: Lower bound of output range.
    :param float out_max: Upper bound of output range.
    :return: Returns value constrained to given range.
    :rtype: float
    """
    if out_min <= out_max:
        return max(min(x, out_max), out_min)
    return min(max(x, out_max), out_min)


class Filter():
    """Implements the Alpha Beta Filter,
    for more information please see this reference;
    https://en.wikipedia.org/wiki/Alpha_beta_filter
    """
    def __init__(self, dt=0.5, alpha=0.005, beta=0.0):
        self.dt = dt
        self.alpha = alpha
        self.beta = beta

        self.reset()

    def filter(self, xm):
        """Filters a signal.

        Args:
            xm (_type_): Signal you want to filter.

        Returns:
            _type_: A filtered signal.
        """
        xk = self.xk_1 + (self.vk_1 * self.dt)
        vk = self.vk_1

        rk = xm - xk

        xk += self.alpha * rk
        vk += (self.beta * rk) / self.dt

        self.xk_1 = xk
        self.vk_1 = vk

        return xk

    def reset(self):
        """Resets the memory of the filter.
        """
        self.xk_1 = 0.0
        self.vk_1 = 0.0


class Robot:
    """Provides functions to manipulate robot.
    """
    def __init__(self):
        """Variables for the motor and servos.
        """
        # assigns values for motor and servo variables
        self.left_motor = None
        self.right_motor = None
        self.forklift_motor = None
        self.collection_motor = None
        self.box_servo = None

        self.pwm_freq = 50  # Hertz
        self.min_pulse = 1000  # milliseconds
        self.max_pulse = 2000  # milliseconds
        self.servo_range = 120  # degrees

        self.servo_in_value = 25 # usually 0
        self.servo_out_value = -100 # usually 120

        self.is_servo_holding_modules = True

        self.left_speed = 0
        self.right_speed = 0
        self.speed_filter = Filter()
        self.heading_filter = Filter()

        self.use_filter = False

    def _make_motor(self, addr, reverse=False):
        """Makes a motor.

        Args:
            addr (_type_): Assigns a port to the motor.

        Returns:
            _type_: Created motor.
        """
        motor = vex.Motor(addr, reverse)
        #motor = Motor55(addr, reverse)
        # motor = vex.Pwm(addr)
        return motor

    def _make_servo(self, addr):
        """Makes a servo.

        Args:
            addr (_type_): Assigns a port to the servo.

        Returns:
            _type_: Created servo.
        """
        # sets parameters for the servo
        a_servo = vex.Servo(addr)
        return a_servo

    def drive_base_arcade(self, speed: float, heading: float):
        """Drive's the robot.

        Args:
            speed (float): Desired speed of robot.
            heading (float): Desired heading of robot.

        Raises:
            Exception: Prevents error if motor is not created.
        """
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        if self.use_filter:
            # Filtering speed and heading
            speed_filtered = self.speed_filter.filter(speed)
            heading_filtered = self.heading_filter.filter(heading)

            # constrain the speed values
            self.left_speed = constrain(speed_filtered - heading_filtered, -1.0, 1.0)
            self.right_speed = constrain(speed_filtered + heading_filtered, -1.0, 1.0)
        else:
            self.left_speed = constrain(speed + heading, -1.0, 1.0)
            self.right_speed = constrain(speed - heading, -1.0, 1.0)

        # assigns speed to motors
        left_direction, self.left_speed = get_direction_and_speed(self.left_speed)
        right_direction, self.right_speed = get_direction_and_speed(self.right_speed)
        
        self.left_motor.spin(left_direction, self.left_speed, vex.VelocityUnits.PERCENT)
        self.right_motor.spin(right_direction, self.right_speed, vex.VelocityUnits.PERCENT)
        # self.left_motor.state(100)
        # self.right_motor.state(100)

    def stop_base(self):
        """Stops the robot.

        Raises:
            Exception: Prevents error if motor is not made.
        """
        if not self.left_motor or not self.right_motor:
            raise Exception("Robot not initialized")

        # Resets memory values in speed filter
        self.speed_filter.reset()
        self.heading_filter.reset()
        # assigns motors throttle 0 to stop robot
        self.left_motor.throttle = 0.0
        self.right_motor.throttle = 0.0

    def drive_forklift(self, speed: float):
        """Makes the forklift move.

        Args:
            speed (float): Speed to move the forklift.

        Raises:
            Exception: Prevents error if motor is not made.
        """
        if not self.forklift_motor:
            raise Exception("Robot not initialized")

        speed = constrain(speed, -1.0, 1.0)
        
        direction, speed = get_direction_and_speed(speed)
        
        self.forklift_motor.spin(direction, speed, vex.VelocityUnits.PERCENT)

    def drive_forklift_up(self):
        """Drive forklift up.
        """
        self.drive_forklift(1.0)

    def drive_forklift_down(self):
        """Drive forklift down.
        """
        self.drive_forklift(-1.0)

    def stop_forklift(self):
        """Stops forklift.
        """
        self.drive_forklift(0.0)

    def drive_pushy_thingy(self, speed: float):
        """Makes the pushy_thingy move.

        Args:
            speed (float): Speed to move the pushy_thingy.

        Raises:
            Exception: Prevents error if motor is not made.
        """
        if not self.pushy_thingy:
            raise Exception("Robot not initialized")

        speed = constrain(speed, -1.0, 1.0)
        
        direction, speed = get_direction_and_speed(speed)
        
        self.pushy_thingy.spin(direction, speed, vex.VelocityUnits.PERCENT)

    def drive_pushy_thingy_up(self):
        """Drive pushy_thingy up.
        """
        self.drive_pushy_thingy(1.0)

    def drive_pushy_thingy_down(self):
        """Drive pushy_thingy down.
        """
        self.drive_pushy_thingy(-1.0)

    def stop_pushy_thingy(self):
        """Stops pushy_thingy.
        """
        self.drive_pushy_thingy(0.0)

    def assign_servo_to_angle(self, servo, angle):

        # ensure servo angle is within servo opperating range
        # if angle > self.servo_range or angle < 0:
        #     raise Exception(f"Provided servo angle must be within 0-{self.servo_range}")

        servo.set_position(angle)

    def eject_habitat(self):
        """Makes servo move to eject habitat modules.
        """
        # old code that dispenses the modules
        # moves servo
        # self.box_servo.angle = self.servo_out_value
        # t.sleep(0.3)
        # self.box_servo.angle = self.servo_in_value

        if self.is_servo_holding_modules:
            self.assign_servo_to_angle(self.box_servo, self.servo_out_value)
            self.is_servo_holding_modules = False
        else:
            self.assign_servo_to_angle(self.box_servo, self.servo_in_value)
            self.is_servo_holding_modules = True
            
    def print_joystick(self, brain: vex.Brain):
        brain.screen.clear_screen()
        brain.screen.set_cursor(1,1)
        brain.screen.print(self.left_speed, self.right_speed)

def make_manny(brain: vex.Brain):
    """Assigns ports for motors and servo.

    Args:
        brain (vex.Brain): The area of the port that motors and servos are assigned to.

    Returns:
        _type_: Makes robot.
    """
    # assigns ports for motors and servo
    robot = Robot()
    robot.left_motor = robot._make_motor(Ports.PORT13, False)
    robot.right_motor = robot._make_motor(Ports.PORT12, False)
    robot.forklift_motor = robot._make_motor(Ports.PORT11, True)
    # robot.collection_motor = robot._make_motor(brain.three_wire_port.d)
    robot.box_servo = robot._make_servo(brain.three_wire_port.e)
    robot.pushy_thingy = robot._make_motor(Ports.PORT14)

    return robot

# Brain should be defined by default
brain = vex.Brain()
controller = vex.Controller()

robot = make_manny(brain)

# control task motor with right trigger / shoulder button
controller.buttonR1.pressed(robot.drive_forklift_up)
controller.buttonR1.released(robot.stop_forklift)
controller.buttonR2.pressed(robot.drive_forklift_down)
controller.buttonR2.released(robot.stop_forklift)

# parameter that allows the servo moves
controller.buttonA.released(robot.eject_habitat)

# control task motor with right trigger / shoulder button
controller.buttonL1.pressed(robot.drive_pushy_thingy_up)
controller.buttonL1.released(robot.stop_pushy_thingy)
controller.buttonL2.pressed(robot.drive_pushy_thingy_down)
controller.buttonL2.released(robot.stop_pushy_thingy)

def print_to_brain():
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    
    # print values to brain screen
    brain.screen.set_font(vex.FontType.PROP30)
    brain.screen.print("THE BEST PROGRAMMER")
    brain.screen.next_row()
    brain.screen.print("TO EVER EXIST")
    brain.screen.next_row()
    brain.screen.set_font(vex.FontType.PROP40)
    brain.screen.print(boc1)
    
    # vex.wait(100,MSEC)


print_to_brain()

while True:
    
    # Mix right joystick axes to control both wheels
    speed = map_range(controller.axis3.position(), -100.0, 100.0, -1.0, 1.0)
    heading = map_range(controller.axis1.position(), -100.0, 100.0, -1.0, 1.0)
    robot.drive_base_arcade(speed, heading)
    robot.print_joystick(brain)
    #vex.wait(50,MSEC)