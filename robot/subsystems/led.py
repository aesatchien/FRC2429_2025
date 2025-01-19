import enum
import math
import commands2
import wpilib
from wpilib import AddressableLED
from wpilib import Color, SmartDashboard

import constants


class Led(commands2.Subsystem):
    class Mode(enum.Enum):
        NOTE_LOADED = 'NOTE_LOADED'
        NOTE_SPOTTED = 'NOTE_SPOTTED'
        NONE = 'NONE'

    # temporary indicators (flashing for pickup, strafing, etc)
    class Indicator(enum.Enum):
        READY_SHOOT = 'READY_SHOOT'  # flashing white
        AMP = 'AMP'  # red
        INTAKE = 'INTAKE'  # flashing blue
        PICKUP_COMPLETE = 'AUTO_STRAFE_COMPLETE'  # solid blue
        RAINBOW = 'RAINBOW'
        INTAKE_ON = 'INTAKE_ON'
        CALIBRATION_START = 'CALIBRATION_START'
        CALIBRATION_SUCCESS = 'CALIBRATION_SUCCESS'
        CALIBRATION_FAIL = 'CALIBRATION_FAIL'
        CLIMB = 'CLIMB'
        KILL = 'KILL'
        SHOOTER_ON = 'SHOOTER_ON'
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
                    if self.indicator == Led.Indicator.READY_SHOOT:
                        # flashing yellow
                        freq = 1  # 10 /s > 2x /s
                        cycle = math.floor(self.animation_counter / freq)

                        if cycle % 2 == 0:
                            led.setRGB(0, 0, 0)
                        else:
                            led.setRGB(255, 255, 0)

                    elif self.indicator == Led.Indicator.AMP:
                        # solid red
                        led.setRGB(180, 30, 30)

                    elif self.indicator == Led.Indicator.INTAKE:
                        # solid blue
                        # freq = 1  # 10 /s > 2x /s
                        # cycle = math.floor(self.animation_counter / freq)
                        #
                        # if cycle % 2 == 0:
                        #     led.setRGB(0, 0, 0)
                        # else:
                        led.setRGB(0, 0, 255)

                    elif self.indicator == Led.Indicator.PICKUP_COMPLETE:
                        # solid orange
                        led.setRGB(255, 40, 0)

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


