class Enum:
    pass

class RotationUnits(Enum):
    DEG = 0
    REV = 1
    RAW = 99

class WhiteBalanceMode(Enum):
    pass

class WifiMode(Enum):
    pass

class Ports(Enum):
    PORT1 = 0
    PORT2 = 1
    PORT3 = 2
    PORT4 = 3
    PORT5 = 4
    PORT6 = 5
    PORT7 = 6
    PORT8 = 7
    PORT9 = 8
    PORT10 = 9
    PORT11 = 10
    PORT12 = 11
    PORT13 = 12
    PORT14 = 13
    PORT15 = 14
    PORT16 = 15
    PORT17 = 16
    PORT18 = 17
    PORT19 = 18
    PORT20 = 19
    PORT21 = 20
    PORT22 = 21

class VoltageUnits(Enum):
    VOLT = 0
    MV = 1

class TorqueUnits(Enum):
    NM = 0
    IN_LB = 1

class TemperatureUnits(Enum):
    CELSIUS = 0
    FAHRENHEIT = 1
    PCT = 0xFF

class PercentUnits(Enum):
    PCT = 0

class AnalogUnits(Enum):
    PCT = PercentUnits.PCT
    RANGE_8BIT = 1
    RANGE_10BIT = 2
    RANGE_12BIT = 3
    MV = 4

class CurrentUnits(Enum):
    AMP = 0

class PowerUnits(Enum):
    WATT = 0

class DirectionType(Enum):
    FWD = 0
    REV = 1
    UNDEFINED = 2

class DistanceUnits(Enum):
    MM = 0
    IN = 1
    CM = 2

class TriportPort:
    pass

class TriDevice:
    def __init__(self,  port, type):
        pass


class Device:
    def type(self):
        pass

    def installed(self):
        pass

    def value(self):
        pass

class TriPort(Device):
    port = [TriportPort(), TriportPort(), TriportPort(), TriportPort(), TriportPort(), TriportPort(), TriportPort(), TriportPort()]
    a = port[0]
    b = port[1]
    c = port[2]
    d = port[3]
    e = port[4]
    f = port[5]
    g = port[6]
    h = port[7]

class TurnType(Enum):
    LEFT = 0
    RIGHT = 1

class V5DeviceType(Enum):
    NO_SENSOR = 0
    MOTOR = 2
    LED = 3
    ABS_ENC = 4
    BUMPER = 5
    IMU = 6
    RANGE = 7
    RADIO = 8
    TETHER = 9
    BRAIN = 10
    VISION = 11
    ADI = 12
    GYRO = 0x46
    SONAR = 0x47
    GENERIC = 128
    GENERIC_SERIAL = 129
    UNDEFINED = 255

class VelocityUnits(Enum):
    PCT = PercentUnits.PCT
    RPM = 1
    DPS = 2

class Accelerometer(TriDevice):
    def value(self, analogUnits=AnalogUnits.PCT):
        pass

class AnalogIn(TriDevice):
    def value(self, analogUnits=AnalogUnits.PCT):
        pass

class TimeUnits(Enum):
    SEC = 0
    MSEC = 1

class Timer:
    def time(self, timeUnits=TimeUnits.SEC):
        pass

    def clear(self):
        pass

    @staticmethod
    def system():
        pass

class BrainBattery:
    def capacity(self, percentUnits=PercentUnits.PCT):
        pass

    def temperature(self, temperatureUnits=TemperatureUnits.CELSIUS):
        pass

    def voltage(self, voltageUnits=VoltageUnits.VOLT):
        pass

    def current(self, currentUnits=CurrentUnits.AMP):
        pass

class BrainLcd:
    def set_cursor(self, row, col):
        pass

    def set_font(self, font):
        pass

    def set_pen_width(self, width):
        pass

    def set_origin(self, x, y):
        pass

    def column(self):
        pass

    def row(self):
        pass

    def set_pen_color(self, color):
        pass

    def set_pen_color_hue(self, hue):
        pass

    def set_fill_color(self, color):
        pass

    def set_fill_color_hue(self, color):
        pass

    def get_string_width(self, text):
        pass

    def get_string_height(self, text):
        pass

    def print_(self, text, new_line=True):
        pass

    def print_line(self, number, text):
        pass

    def print_at(self, x, y, opaque, text):
        pass

    def clear_screen(self, color=None):
        pass

    def clear_screen_hue(self, hue):
        pass

    def clear_line(self, number=None, color=None):
        pass

    def clear_line_hue(self, number, hue):
        pass

    def new_line(self):
        pass

    def draw_pixel(self, x, y):
        pass

    def draw_line(self, x1, y1, x2, y2):
        pass

    def draw_rectangle(self, x, y, width, height, color=None):
        pass

    def draw_rectangle_hue(self, x, y, width, height, hue):
        pass

    def draw_circle(self, x, y, radius, color=None):
        pass

    def draw_circle_hue(self, x, y, radius, hue):
        pass

    def draw_bitmap(self, bitmap, x=0, y=0):
        pass

    def draw_eyes(self, eyes):
        pass

    def x_position(self):
        pass

    def y_position(self):
        pass

    def pressing(self):
        pass

    def render(self, vsync_wait=True):
        pass

class Brain(Device):
    screen = BrainLcd()
    battery = BrainBattery()
    three_wire_port = TriPort() #not sure this is the class
    timer = Timer()

class BrakeType:
    COAST = 0
    BRAKE = 1
    HOLD = 2
    UNDEFINED = 3

class Bumper(TriDevice):
    def value(self):
        pass

    def pressing(self):
        pass

class Color:
    def set_rgb(self, r, g, b):
        pass

    def set_value(self, color_value):
        pass

    def set_hsv(self, hue, saturation, value):
        pass

    def set_web(self, web_color_str):
        pass

    def value(self):
        pass

    def is_transparent(self):
        pass

    @staticmethod
    def from_rgb(r, g, b):
        pass

    @staticmethod
    def from_hsv(h, s, v):
        pass

    @staticmethod
    def from_web(web_color_str):
        pass

    BLACK = None
    WHITE = None
    RED = None
    GREEN = None
    BLUE = None
    YELLOW = None
    ORANGE = None
    PURPLE = None
    CYAN = None
    TRANSPARENT = None

class Competition:
    def is_enabled(self):
        pass

    def is_driver_control(self):
        pass

    def is_autonomous(self):
        pass

    def is_competition_switch(self):
        pass

    def is_field_control(self):
        pass

    def autonomous(self, f):
        pass

    def drivercontrol(self, f):
        pass

class ControllerAxis:
    def value(self):
        pass

    def position(self, percentUnits=PercentUnits.PCT):
        pass

class ControllerButton:
    def pressing(self):
        pass

class ControllerLcd:
    def set_cursor(self, row, col):
        pass

    def print_(self, text, new_line=True):
        pass

    def clear_screen(self):
        pass

    def clear_line(self, number=None):
        pass

    def new_line(self):
        pass

class ControllerType(Enum):
    PRIMARY = 0
    PARTNER = 1

class Controller(Device):
    def __init__(self, controller_type):
        pass

    def rumble(self, rumblePattern):
        pass

    def set_deadband(self, deadband, percentUnits=PercentUnits.PCT):
        pass

    def deadband(self, percentUnits=PercentUnits.PCT):
        pass

    buttonL1 = ControllerButton()
    buttonL2 = ControllerButton()
    buttonR1 = ControllerButton()
    buttonR2 = ControllerButton()
    buttonUp = ControllerButton()
    buttonDown = ControllerButton()
    buttonLeft = ControllerButton()
    buttonRight = ControllerButton()
    buttonX = ControllerButton()
    buttonB = ControllerButton()
    buttonY = ControllerButton()
    buttonA = ControllerButton()
    axis1 = ControllerAxis()
    axis2 = ControllerAxis()
    axis3 = ControllerAxis()
    axis4 = ControllerAxis()
    screen = ControllerLcd()

class DetectionMode(Enum):
    pass

class Devices:
    def get(self, index):
        pass

    def type(self, index):
        pass

    def number(self):
        pass

    def number_of(self, type):
        pass

class DigitalIn(TriDevice):
    def value(self):
        pass

    def is_set(self):
        pass

class DigitalOut(TriDevice):
    def value(self):
        pass

    def set(self, is_set):
        pass

    def is_set(self):
        pass

class Drivetrain:
    def set_velocity(self, velocity, velocityUnits=VelocityUnits.PCT):
        pass

    def drive(self, directionType, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def drive_for(self, directionType, distance, distanceUnits=DistanceUnits.MM, velocity=None,
                  velocityUnits=VelocityUnits.PCT, waitForCompletion=True):
        pass

    def turn(self, turnType, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def turn_for(self, turnType, angle, angleUnits=RotationUnits.DEG, velocity=None, velocityUnits=VelocityUnits.PCT,
                 waitForCompletion=True):
        pass

    def start_drive_for(self, directionType, distance, distanceUnits=DistanceUnits.MM, velocity=None,
                        velocityUnits=VelocityUnits.PCT):
        pass

    def start_turn_for(self, turnType, angle, angleUnits=RotationUnits.DEG, velocity=None,
                       velocityUnits=VelocityUnits.PCT, waitForCompletion=True):
        pass

    def arcade(self, drivePower, turnPower, percentUnit=PercentUnits.PCT):
        pass

    def stop(self, brakeType=None):
        pass

    def set_timeout(self, time, timeUnits=TimeUnits.SEC):
        pass

    def timeout(self, timeUnits=TimeUnits.SEC):
        pass

    def did_timeout(self):
        pass

    def is_spinning(self):
        pass

    def is_done(self):
        pass

class Encoder(TriDevice):
    def value(self):
        pass

    def reset_rotation(self):
        pass

    def set_rotation(self, value, rotationUnits=RotationUnits.DEG):
        pass

    def rotation(self, rotationUnits=RotationUnits.DEG):
        pass

    def velocity(self, velocityUnits=VelocityUnits.PCT):
        pass

class Eyes(Enum):
    BLACK_LEFT = 1
    BLACK_DOWN = 2
    BLACK_RESTING = 3
    BLACK_RIGHT = 4
    BLACK_STRAIGHT = 5
    BLUE_ANGRY = 6
    BLUE_RESTING = 7
    BLUE_RESTING_L = 8
    BLUE_RESTING_R = 9
    BLUE_SAD = 10
    BLUE_SQUINTING = 11
    GREEN_DOWN = 12
    GREEN_LEFT = 13
    GREEN_RESTING = 14
    GREEN_RIGHT = 15
    GREEN_UP = 16
    PURPLE_DOWN = 17
    PURPLE_LEFT = 18
    PURPLE_RESTING = 19
    PURPLE_RIGHT = 20
    PURPLE_UP = 21

class Font(Enum):
    MONO_20 = 0
    MONO_30 = 1
    MONO_40 = 2
    MONO_60 = 3
    PROP_20 = 4
    PROP_30 = 5
    PROP_40 = 6
    PROP_60 = 7
    MONO_15 = 8
    MONO_12 = 9

class GearSetting(Enum):
    RATIO36_1 = 0
    RATIO18_1 = 1
    RATIO6_1 = 2

class Gyro(TriDevice):
    def value(self, rotationUnits=RotationUnits.DEG):
        pass

    def start_calibration(self, value=0):
        pass

    def is_calibrating(self):
        pass

class Led(DigitalOut):
    def on(self):
        pass

    def off (self):
        pass

class LedMode(Enum):
    pass

class Light(TriDevice):
    def value(self, analogUnits=AnalogUnits.PCT):
        pass

class Limit(TriDevice):
    def value(self):
        pass

    def pressing(self):
        pass

class Line(TriDevice):
    def value(self, analogUnits=AnalogUnits.PCT):
        pass

class Motor(Device):
    def __init__(self, port, gear_setting, is_reversed):
        pass

    def set_reversed(self, isReversed):
        pass

    def set_velocity(self, velocity, velocityUnits=VelocityUnits.PCT):
        pass

    def set_stopping(self, brakeType):
        pass

    def reset_rotation(self):
        pass

    def set_rotation(self, value, rotationUnits=RotationUnits.DEG):
        pass

    def set_timeout(self, time, timeUnits=TimeUnits.SEC):
        pass

    def timeout(self, timeUnits=TimeUnits.SEC):
        pass

    def did_timeout(self):
        pass

    def spin(self, direction, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def spin_with_voltage(self, direction, voltage, voltageUnits=VoltageUnits.VOLT):
        pass

    def rotate_to(self, rotation, rotationUnits=RotationUnits.DEG, velocity=None, velocityUnits=VelocityUnits.PCT,
                  waitForCompletion=True):
        pass

    def rotate_for(self, direction, rotation, rotationUnits=RotationUnits.DEG, velocity=None,
                   velocityUnits=VelocityUnits.PCT, waitForCompletion=True):
        pass

    def rotate_for_time(self, direction, time, timeUnits=TimeUnits.SEC, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def start_rotate_to(self, rotation, rotationUnits=RotationUnits.DEG, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def start_rotate_for(self, direction, rotation, rotationUnits=RotationUnits.DEG, velocity=None,
                         velocityUnits=VelocityUnits.PCT):
        pass

    def is_spinning(self):
        pass

    def is_done(self):
        pass

    def stop(self, brakeType=None):
        pass

    def set_max_torque(self, value, torqueUnits=TorqueUnits.NM):
        pass

    def set_max_torque_percent(self, value, percentUnits=PercentUnits.PCT):
        pass

    def set_max_torque_current(self, value, currentUnits=CurrentUnits.AMP):
        pass

    def direction(self):
        pass

    def rotation(self, rotationUnits=RotationUnits.DEG):
        pass

    def velocity(self, velocityUnits=VelocityUnits.PCT):
        pass

    def current(self, currentUnits=CurrentUnits.AMP):
        pass

    def voltage(self, voltageUnits=VoltageUnits.VOLT):
        pass

    def power(self, powerUnits=PowerUnits.WATT):
        pass

    def torque(self, torqueUnits=TorqueUnits.NM):
        pass

    def efficiency(self, percentUnits=PercentUnits.PCT):
        pass

    def temperature(self, temperatureUnits=TemperatureUnits.CELSIUS):
        pass

class Motor29(Device):
    def set_velocity(self, velocity, percentUnits=PercentUnits.PCT):
        pass

    def set_reversed(self, is_reversed):
        pass

    def spin(self, direction, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def stop(self):
        pass

class MotorVictor(Device):
    def set_velocity(self, velocity, percentUnits=PercentUnits.PCT):
        pass

    def set_reversed(self, is_reversed):
        pass

    def spin(self, direction, velocity=None, velocityUnits=VelocityUnits.PCT):
        pass

    def stop(self):
        pass

class Pneumatics(DigitalOut):
    def open(self):
        pass

    def close(self):
        pass

class Pot(TriDevice):
    def value(self, rotationUnits=RotationUnits.DEG):
        pass

class PwmOut(TriDevice):
    def set_state(self, value, percentUnits=PercentUnits.PCT):
        pass

class Servo(TriDevice):
    def set_position(self, value, rotationUnits=RotationUnits.DEG):
        pass

class Sonar(TriDevice):
    def value(self):
        pass

    def distance(self, distanceUnits=DistanceUnits.MM):
        pass

class VisionCode:
    pass

class VisionObject:
    id = 0
    originX = 0
    originY = 0
    centerX = 0
    centerY = 0
    width = 0
    height = 0
    angle = 0.
    exists = False

class VisionSignature:
    def __init__(self, id, uMin, uMax, uMean, vMin, vMax, vMean, range, type):
        pass

    id = 0
    uMin = 0
    uMax = 0
    uMean = 0
    vMin = 0
    vMax = 0
    vMean = 0
    range = 0.
    rgb = 0
    type = 0

class Vision(Device):
    def __init__(self, port):
        pass

    def take_snapshot(self, id_or_sig_or_code, count=None):
        pass

    def set_signature(self, signature):
        pass

    def set_mode(self, mode):
        pass

    def get_mode(self):
        pass

    def set_brightness(self, value):
        pass

    def get_brightness(self):
        pass

    def set_white_balance_mode(self, mode):
        pass

    def get_white_balance_mode(self):
        pass

    def set_white_balance_values(self, color):
        pass

    def get_white_balance_values(self):
        pass

    def set_led_mode(self, mode):
        pass

    def get_led_mode(self):
        pass

    def set_led_brightness(self, percent):
        pass

    def get_led_brightness(self):
        pass

    def set_led_color(self, red, green, blue):
        pass

    def get_led_color(self):
        pass

    def set_wifi_mode(self, mode):
        pass

    def get_wifi_mode(self):
        pass

    object_count = 0
    largest_object = VisionObject()
    objects = []

