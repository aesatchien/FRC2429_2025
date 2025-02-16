from enum import Enum
import time  # import time for precise timing
import commands2
from wpilib import AddressableLED
from wpilib import SmartDashboard, Color  # can i make use of color at some point?
import constants

# TODO - make the frequencies actual times per second - so divide by the LED update rate (currently 10x per second)


class Led(commands2.Subsystem):
    """ LED Subsystem
    This subsystem uses modes (constant settings for the robot) and indicators (settings meant to be temporary)
    to communicate robot states to the driver
    Modes: NONE, CORAL, ALGAE
    INDICATORS: various colors or animations
    Has getters and setters for the modes and indicators
    Provides a dictionary of the modes and indicators for the RobotContainer so they can be placed on the dash
    Updates the LEDs 10 times per second (every 5 robot cycles)
    """
    class Indicator(Enum):
        """ Indicator class is for showing conditions or animations """
        # animated indicators
        kRAINBOW = {'name': "RAINBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 5, "duty_cycle": None,
                    'animation_data': [(int(180 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kCOOLBOW = {'name': "COOLBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 20, "duty_cycle": None,
                    'animation_data': [(int(60 + 90 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kHOTBOW = {'name': "HOTBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 20, "duty_cycle": None,
                   'animation_data': [(int(150 + 60 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kPOLKA = {'name': "POLKA", "on_color": None, "off_color": None, "animated": True, "frequency": 2, "duty_cycle": None,
                  'animation_data': [(255, 255, 255) if i % 2 == 0 else (0, 0, 0) for i in range(constants.k_led_count)], 'use_hsv': False, 'use_mode': True}
        # non-animated indicators
        kSUCCESS = {'name': "SUCCESS", "on_color": [0, 255, 0], "off_color": [0, 0, 0],             "animated": False, "frequency": 3, "duty_cycle": 0.25, 'use_mode': False}
        kSUCCESSFLASH = {'name': "SUCCESS + MODE", "on_color": [0, 255, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': True}
        kFAILURE = {'name': "FAILURE", "on_color": [255, 0, 0], "off_color": [0, 0, 0],             "animated": False, "frequency": 3, "duty_cycle": 0.75, 'use_mode': False}
        kFAILUREFLASH = {'name': "FAILURE + MODE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': True}
        kNONE = {'name': "NONE", "on_color": [255, 0, 0], "off_color": [0, 0, 0],                   "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': False}

    class Mode(Enum):
        """ Mode class is for showing robot's current scoring mode and is the default during teleop """
        kCORAL = {'name': "CORAL", "on_color": [255, 255, 255], "off_color": [0, 0, 0], "animated": False, "frequency": None, "duty_cycle": None}
        kALGAE = {'name': "ALGAE", "on_color": [0, 180, 180], "off_color": [0, 0, 0], "animated": False, "frequency": None, "duty_cycle": None}
        kNONE = {'name': "NONE", "on_color": [180, 0, 180], "off_color": [0, 0, 0], "animated": False, "frequency": None, "duty_cycle": None}


    def __init__(self, container):
        super().__init__()
        self.setName('Led')
        self.container = container  # at the moment LED may want to query other subsystems, but this is not clean
        # try to start all the subsystems on a different count so they don't all do the periodic updates at the same time
        self.counter = 1
        self.animation_counter = 0
        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.indicators_dict = {indicator.value["name"]: indicator for indicator in self.Indicator}
        self.modes_dict = {mode.value["name"]: mode for mode in self.Mode}
        self.last_toggle_time = time.monotonic()  # Tracks the last toggle time
        self.toggle_state = False  # Keeps track of the current on/off state

        # necessary initialization for the LED strip
        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(0, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        # initialize modes and indicators
        self.mode = self.Mode.kNONE
        self.prev_mode = self.Mode.kNONE
        self.indicator = Led.Indicator.kPOLKA

        self.set_mode(self.Mode.kNONE)
        self.set_indicator(self.Indicator.kNONE)

    def set_mode(self, mode) -> None:
        self.prev_mode = self.mode
        self.mode = mode
        SmartDashboard.putString('led_mode', self.mode.value['name'])

    def get_mode(self):
        return self.mode

    def set_indicator(self, indicator) -> None:
        self.indicator = indicator
        SmartDashboard.putString('led_indicator', self.indicator.value['name'])

    def set_indicator_with_timeout(self, indicator: Indicator, timeout: float) -> commands2.ParallelRaceGroup:
        return commands2.StartEndCommand(
            lambda: self.set_indicator(indicator),
            lambda: self.set_indicator(Led.Indicator.kNONE),
        ).withTimeout(timeout)

    def periodic(self):
        if self.counter % 5 == 0:  # Execute every 5 cycles (10Hz update rate)
            current_time = time.monotonic()  # Current time in seconds
            time_since_toggle = current_time - self.last_toggle_time

            if self.indicator != self.Indicator.kNONE:
                if not self.indicator.value["animated"]:  # Non-animated indicators
                    frequency = self.indicator.value["frequency"]
                    period = 1 / frequency  # Period for one cycle (on + off)

                    # Calculate duty cycle timing
                    duty_cycle = self.indicator.value.get("duty_cycle", 0.5)  # Default to 50% if not specified
                    on_time = period * duty_cycle
                    off_time = period * (1 - duty_cycle)

                    if self.toggle_state and time_since_toggle >= on_time:
                        self.toggle_state = False
                        self.last_toggle_time = current_time
                    elif not self.toggle_state and time_since_toggle >= off_time:
                        self.toggle_state = True
                        self.last_toggle_time = current_time

                    # Determine color based on toggle state
                    if self.toggle_state:
                        color = self.indicator.value["on_color"]
                    else:
                        color = self.mode.value["on_color"] if self.indicator.value["use_mode"] else self.indicator.value["off_color"]

                    for i in range(constants.k_led_count):  # Apply the color to all LEDs
                        self.led_data[i].setRGB(*color)

                else:  # Handle animated indicators
                    data = self.indicator.value["animation_data"]
                    if time_since_toggle > 1 / self.indicator.value["frequency"]:
                        self.animation_counter +=1
                        self.last_toggle_time = current_time

                    shift = self.animation_counter % self.led_count
                    shifted_data = data[shift:] + data[:shift]
                    if self.indicator.value["use_hsv"]:
                        [self.led_data[i].setHSV(*shifted_data[i]) for i in range(self.led_count)]
                    else:
                        [self.led_data[i].setRGB(*shifted_data[i]) for i in range(self.led_count)]

            else:  # Handle mode-only LEDs - they do not toggle
                color = self.mode.value["on_color"]
                for i in range(constants.k_led_count):
                    self.led_data[i].setRGB(*color)

            self.led_strip.setData(self.led_data)  # Send LED updates

        self.counter += 1  # Increment the main counter
