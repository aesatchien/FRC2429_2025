import math
import commands2
import wpilib
from wpilib import AddressableLED
from wpilib import SmartDashboard, Color  # can i make use of color at some point?
import constants

# TODO - make the frequencies actual times per second - so divide by the LED update rate (currently 10x per second)


class Led(commands2.Subsystem):

    class Indicator:
        """ Indicator class is for showing conditions or animations """
        white = (255, 255, 255)
        black = (0, 0, 0)
        # the animated classes need to have their data specified here - so you may have to think about it a bit
        # RAINBOW has an illusion of backwards and forwards depending on if shift the data positive or negative
        kRAINBOW = {'name': "RAINBOW", "on_color": [0, 0, 0], "off_color": [0, 0, 0], "animated": True, "frequency": 0.5, "flash_mod": 2,
                    'animation_data': [(int(180 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True}
        kCOOLBOW = {'name': "COOLBOW", "on_color": [0, 0, 0], "off_color": [0, 0, 0], "animated": True, "frequency": 1, "flash_mod": 2,
                    'animation_data': [(int(60 + 90 * (i / constants.k_led_count)), 255, 255) for i in range(constants.k_led_count)], 'use_hsv': True}
        kPOLKA = {'name': "POLKA", "on_color": [0, 0, 0], "off_color": [0, 0, 0], "animated": True, "frequency": 0.25, "flash_mod": 2,
                  'animation_data': [(255, 255, 255) if i % 2 == 0 else (0, 0, 0) for i in range(constants.k_led_count)], 'use_hsv': False}
        # classes that are not animated have their data generated on the fly
        kSUCCESS = {'name': "SUCCESS", "on_color": [0, 255, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}
        kFAILURE = {'name': "FAILURE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}
        kNONE = {'name': "NONE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}

    class Mode:
        """ Mode class is for showing robot's current scoring mode and is the default during teleop """
        kCORAL = {'name': "CORAL", "on_color": [255, 255, 255], "off_color": [0, 0, 0], "animated": True, "frequency": 1, "flash_mod": 2}
        kALGAE = {'name': "ALGAE", "on_color": [0, 180, 180], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}
        kNONE = {'name': "NONE", "on_color": [180, 0, 180], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}


    def __init__(self, container):
        super().__init__()
        self.setName('Led')
        self.container = container  # at the moment LED may want to query other subsystems, but this is not clean
        self.counter = 0
        self.animation_counter = 0  # will not be necessary after refactor
        # need to fix this kludge
        self.indicators = [self.Indicator.kNONE, self.Indicator.kSUCCESS, self.Indicator.kFAILURE, self.Indicator.kRAINBOW, self.Indicator.kCOOLBOW, self.Indicator.kPOLKA]
        self.indicators_dict = {indicator['name']: indicator for indicator in self.indicators}
        self.modes = [self.Mode.kNONE, self.Mode.kALGAE, self.Mode.kCORAL]
        self.modes_dict = {mode['name']: mode for mode in self.modes}

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
        SmartDashboard.putString('led_mode', self.mode['name'])

    def get_mode(self):
        return self.mode

    def set_indicator(self, indicator) -> None:
        self.indicator = indicator
        SmartDashboard.putString('led_indicator', self.indicator['name'])

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

                if not self.indicator["animated"]:  # no animation
                    if self.indicator["frequency"] == 0:  # solid color
                        color = self.indicator["on_color"]
                        # [self.led_data[i].setRGB(*indicator_parameters["on_color"]) for i in range(constants.k_led_count)]

                    else:  # flashing color
                        cycle = math.floor(self.animation_counter / self.indicator["frequency"])

                        if cycle % self.indicator["flash_mod"] == 0:
                            color = self.indicator["off_color"]

                        else:
                            color = self.indicator["on_color"]

                    for i in range(constants.k_led_count):  # set all the LEDs to the same color
                        self.led_data[i].setRGB(*color)

                else:  # special animation cases -
                    data = self.indicator['animation_data']
                    # a frequency < 1 slows down the animation
                    # TODO: control the direction of the animation by choosing a ±1 prefactor for the shift, maybe put in the dictionary
                    shift = int(self.animation_counter * self.indicator['frequency']) % self.led_count
                    shifted_data = data[shift:] + data[:shift]  # rotate data efficiently
                    if self.indicator['use_hsv']:
                        [self.led_data[i].setHSV(*shifted_data[i]) for i in range(self.led_count)]
                    else:
                        [self.led_data[i].setRGB(*shifted_data[i]) for i in range(self.led_count)]

            else:  # mode colors
                pass  # todo programmatically figure out what mode to do
                color = self.mode["on_color"]

                for i in range(constants.k_led_count):  # set all the LEDs to the same color
                    self.led_data[i].setRGB(*color)

            self.led_strip.setData(self.led_data)  # send the colors to the LEDs

        self.counter += 1


