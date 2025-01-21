from enum import Enum
import math
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
        # the animated classes need to have their data specified here - so you may have to think about it a bit
        # RAINBOW has an illusion of backwards and forwards depending on if shift the data positive or negative
        kRAINBOW = {'name': "RAINBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 5, "flash_mod": None,
                    'animation_data': [(int(180 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kCOOLBOW = {'name': "COOLBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 10, "flash_mod": None,
                    'animation_data': [(int(60 + 90 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kPOLKA = {'name': "POLKA", "on_color": None, "off_color": None, "animated": True, "frequency": 2.5, "flash_mod": None,
                  'animation_data': [(255, 255, 255) if i % 2 == 0 else (0, 0, 0) for i in range(constants.k_led_count)], 'use_hsv': False, 'use_mode': True}
        # classes that are not animated have their data generated on the fly
        kSUCCESS = {'name': "SUCCESS", "on_color": [0, 255, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2, 'use_mode': False}
        kSUCCESSFLASH = {'name': "SUCCESS + MODE", "on_color": [0, 255, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2, 'use_mode': True}
        kFAILURE = {'name': "FAILURE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2, 'use_mode': False}
        kFAILUREFLASH = {'name': "FAILURE + MODE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2, 'use_mode': True}
        kNONE = {'name': "NONE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2, 'use_mode': False}

    class Mode(Enum):
        """ Mode class is for showing robot's current scoring mode and is the default during teleop """
        kCORAL = {'name': "CORAL", "on_color": [255, 255, 255], "off_color": [0, 0, 0], "animated": True, "frequency": 10, "flash_mod": 2}
        kALGAE = {'name': "ALGAE", "on_color": [0, 180, 180], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2}
        kNONE = {'name': "NONE", "on_color": [180, 0, 180], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "flash_mod": 2}


    def __init__(self, container):
        super().__init__()
        self.setName('Led')
        self.container = container  # at the moment LED may want to query other subsystems, but this is not clean
        # try to start all the subsystems on a different count so they don't all do the periodic updates at the same time
        self.counter = 1
        self.animation_counter = 0  # will not be necessary after refactor
        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.indicators_dict = {indicator.value["name"]: indicator for indicator in self.Indicator}
        self.modes_dict = {mode.value["name"]: mode for mode in self.Mode}

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

        # update LEDs
        if self.counter % 5 == 0:
            self.animation_counter += 1  # need to slowly update the internal counter

            if self.indicator != self.Indicator.kNONE:

                if not self.indicator.value["animated"]:  # no animation
                    if self.indicator.value["frequency"] == 0:  # solid color
                        color = self.indicator.value["on_color"]

                    else:  # flashing color
                        cycle = math.floor(self.animation_counter * self.indicator.value["frequency"])

                        if cycle % self.indicator.value["flash_mod"] == 0:  # off color
                            # allow us to use the mode value for off instead of the default in the indicator
                            color = self.mode.value["on_color"] if self.indicator.value['use_mode'] else self.indicator.value["off_color"]

                        else:  # on color
                            color = self.indicator.value["on_color"]

                    for i in range(constants.k_led_count):  # set all the LEDs to the same color
                        self.led_data[i].setRGB(*color)

                else:  # special animation cases -
                    data = self.indicator.value['animation_data']
                    # a frequency < 1 slows down the animation
                    # TODO: control the direction of the animation by choosing a ±1 prefactor for the shift, maybe put in the dictionary
                    shift = int(self.animation_counter * self.indicator.value['frequency']) % self.led_count
                    shifted_data = data[shift:] + data[:shift]  # rotate data efficiently
                    if self.indicator.value['use_hsv']:
                        [self.led_data[i].setHSV(*shifted_data[i]) for i in range(self.led_count)]
                    else:
                        [self.led_data[i].setRGB(*shifted_data[i]) for i in range(self.led_count)]

            else:  # mode colors
                pass  # todo programmatically figure out what mode to do
                color = self.mode.value["on_color"]

                for i in range(constants.k_led_count):  # set all the LEDs to the same color
                    self.led_data[i].setRGB(*color)

            self.led_strip.setData(self.led_data)  # send the colors to the LEDs

        self.counter += 1


