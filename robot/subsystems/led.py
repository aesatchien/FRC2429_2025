import enum
import math
import commands2
import wpilib
from wpilib import AddressableLED
from wpilib import Color, SmartDashboard

import constants


class Led(commands2.Subsystem):
    indictator_dict = {
        "RAINBOW": {"on_color": [0, 0, 0], "off_color": [0, 0, 0], "animated": True, "frequency": 1, "flash_mod": 2},
        "SUCCESS": {"on_color": [0, 255, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2},
        "FAILURE": {"on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}
    }

    mode_dict = {
        "CORAL": {"on_color": [255, 255, 255], "off_color": [0, 0, 0], "animated": True, "frequency": 1, "flash_mod": 2},
        "ALGAE": {"on_color": [0, 180, 180], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2},
        "NONE": {"on_color": [180, 0, 180], "off_color": [0, 0, 0], "animated": False, "frequency": 1, "flash_mod": 2}
    }

    class Mode(enum.Enum):
        CORAL = 'CORAL'
        ALGAE = 'ALGAE'
        NONE = 'NONE'

    # temporary indicators (flashing for pickup, strafing, etc)
    class Indicator(enum.Enum):
        INTAKE = 'INTAKE'  # flashing blue
        RAINBOW = 'RAINBOW'
        CLIMB = 'CLIMB'
        SUCCESS = 'SUCCESS'
        FAILURE = 'FAILURE'
        KILL = 'KILL'
        NONE = 'NONE'

        POLKA = 'POLKA' # black and white spots looping around led string

    def __init__(self, container):
        super().__init__()
        self.setName('Led')
        self.container = container
        self.counter = 0
        self.animation_counter = 0

        self.polka_counter = 1

        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(0, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        self.mode = self.Mode.NONE

        #self.indicator = Led.Indicator.NONE
        self.indicator = Led.Indicator.POLKA

    def set_mode(self, mode: Mode) -> None:
        self.prev_mode = self.mode
        self.mode = mode

    def get_mode(self) -> Mode:
        return self.mode

    def set_indicator(self, indicator) -> None:
        self.indicator = indicator

    def set_indicator_with_timeout(self, indicator: Indicator, timeout: float) -> commands2.ParallelRaceGroup:
        return commands2.StartEndCommand(
            lambda: self.set_indicator(indicator),
            lambda: self.set_indicator(Led.Indicator.NONE),
        ).withTimeout(timeout)

    def new_periodic(self):

        # update LEDs
        if self.counter % 5 == 0:
            SmartDashboard.putString('led_mode', self.mode.value)
            SmartDashboard.putString('led_indicator', self.indicator.value)

            self.animation_counter += 1

            indicator_parameters = self.indictator_dict[self.indicator.value]
            mode_parameters = self.mode_dict[self.mode.value]

            if self.indicator.value is not None:

                if not indicator_parameters["animated"]:
                    if indicator_parameters["frequency"] == 0:  # solid color
                        color = indicator_parameters["on_color"]
                        # [self.led_data[i].setRGB(*indicator_parameters["on_color"]) for i in range(constants.k_led_count)]

                    else:  # flashing color
                        cycle = math.floor(self.animation_counter / indicator_parameters["frequency"])

                        if cycle % indicator_parameters["flash_mod"] == 0:
                            color = indicator_parameters["off_color"]

                        else:
                            color = indicator_parameters["on_color"]

                    for i in range(constants.k_led_count):  # set all the LEDs to the same color
                        self.led_data[i].setRGB(*color)

                else:  # special animation cases
                    pass

            else:  # mode colors
                pass  # todo programatically figure out what mode to do
                color = mode_parameters["on_color"]

                for i in range(constants.k_led_count):  # set all the LEDs to the same color
                    self.led_data[i].setRGB(*color)

            self.led_strip.setData(self.led_data)  # send the colors to the LEDs


    def periodic(self) -> None:

        # update LEDs
        if self.counter % 5 == 0:
            SmartDashboard.putString('led_mode', self.mode.value)
            SmartDashboard.putString('led_indicator', self.indicator.value)

            # advertise our state to the dash
            # SmartDashboard.putBoolean('cone_selected', self.mode == self.Mode.RING)

            self.animation_counter += 1

            for i in range(constants.k_led_count):
                led = self.led_data[i]

                self.polka_counter *= -1

                # check if there is an indicator, and override
                if self.indicator != Led.Indicator.NONE:
                    if self.indicator == Led.Indicator.INTAKE:
                        # solid blue
                        # freq = 1  # 10 /s > 2x /s
                        # cycle = math.floor(self.animation_counter / freq)
                        #
                        # if cycle % 2 == 0:
                        #     led.setRGB(0, 0, 0)
                        # else:
                        led.setRGB(0, 0, 255)


                    elif self.indicator == Led.Indicator.RAINBOW: # Haochen emote
                        # rainbow
                        hue = (i + self.animation_counter) % constants.k_led_count
                        hue /= constants.k_led_count
                        hue *= 180

                        led.setHSV(math.floor(hue), 255, 255)

                    elif self.indicator == Led.Indicator.INTAKE_ON: # Haochen emote
                        # solid orange
                        # freq = 2  # 10 /s > 2x /s
                        # cycle = math.floor(self.animation_counter / freq)

                        # if cycle % 2 == 0:
                        #     led.setRGB(0, 0, 0)
                        # else:

                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(255, 40, 0)
                        else:
                            if self.container.vision.target_available("orange"):
                                led.setRGB(128, 128, 255)
                            else:
                                led.setRGB(0, 0, 0)

                    elif self.indicator == Led.Indicator.SHOOTER_ON:
                        led.setRGB(0, 255, 0)

                    elif self.indicator == Led.Indicator.KILL:
                        led.setRGB(200, 0, 255)

                    elif self.indicator == Led.Indicator.CALIBRATION_START:
                        led.setRGB(0, 255, 0)

                    elif self.indicator == Led.Indicator.CALIBRATION_SUCCESS:
                        # flashing green
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(0, 255, 0)

                    elif self.indicator == Led.Indicator.CALIBRATION_FAIL:
                        # flashing red
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(255, 0, 0)

                    elif self.indicator == Led.Indicator.CLIMB:
                        # flashing pink
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            # led.setRGB(255, 192, 203)  # probably too bright - looks white - CJH 03292024
                            led.setRGB(220, 172, 183)

                    elif self.indicator == Led.Indicator.POLKA:
                        # circling white and black spots
                        if self.polka_counter == 1:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(123, 123, 123)

                        if i == 0:
                            self.polka_counter *= -1

                else:
                    if self.container.shooter.is_ring_loaded():
                        self.mode = Led.Mode.NOTE_LOADED
                    elif self.container.vision.target_available("orange"):
                        self.mode = Led.Mode.NOTE_SPOTTED
                    else:
                        self.mode = Led.Mode.NONE

                    if self.mode == Led.Mode.NOTE_LOADED:
                        led.setRGB(0, 128, 0)
                    elif self.mode == Led.Mode.NOTE_SPOTTED:
                        led.setRGB(128, 128, 255)
                    elif self.mode == Led.Mode.NONE:
                        led.setRGB(0, 0, 0)
                    # if self.mode == Led.Mode.RING:
                        # solid none
                        # led.setRGB(0, 0, 0)

                    # elif self.mode == Led.Mode.CUBE:
                    #     # solid purple
                    #     led.setRGB(255, 0, 255)


            self.led_strip.setData(self.led_data)

        self.counter += 1


