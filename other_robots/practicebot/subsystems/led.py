from enum import Enum
import time  # import time for precise timing
import commands2
from wpilib import AddressableLED
from wpilib import SmartDashboard, Timer
import ntcore
import constants
from subsystems.robot_state import RobotState

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
                    'animation_data': [(int(180 * (i / constants.LedConstants.k_led_count)), 255, 255) for i in range(constants.LedConstants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kCOOLBOW = {'name': "COOLBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 10, "duty_cycle": None,
                    'animation_data': [(int(60 + 90 * (i / constants.LedConstants.k_led_count)), 255, 255) for i in range(constants.LedConstants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kHOTBOW = {'name': "HOTBOW", "on_color": None, "off_color": None, "animated": True, "frequency": 10, "duty_cycle": None,
                   'animation_data': [(int(150 + 60 * (i / constants.LedConstants.k_led_count)), 255, 255) for i in range(constants.LedConstants.k_led_count)], 'use_hsv': True, 'use_mode': False}
        kPOLKA = {'name': "POLKA", "on_color": None, "off_color": None, "animated": True, "frequency": 2, "duty_cycle": None,
                  'animation_data': [(255, 255, 255) if i % 2 == 0 else (0, 0, 0) for i in range(constants.LedConstants.k_led_count)], 'use_hsv': False, 'use_mode': False}
        # non-animated indicators
        kSUCCESS = {'name': "SUCCESS", "on_color": [0, 255, 0], "off_color": [0, 0, 0],             "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': False}
        kSUCCESSFLASH = {'name': "SUCCESS + MODE", "on_color": [0, 255, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': True}
        kWHITEFLASH = {'name': "SUCCESS + MODE", "on_color": [255, 255, 255], "off_color": [0, 0, 0], "animated": False, "frequency": 10, "duty_cycle": 0.5, 'use_mode': False}
        kFAILURE = {'name': "FAILURE", "on_color": [255, 0, 0], "off_color": [0, 0, 0],             "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': False}
        kFAILUREFLASH = {'name': "FAILURE + MODE", "on_color": [255, 0, 0], "off_color": [0, 0, 0], "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': True}
        kNONE = {'name': "NONE", "on_color": [255, 0, 0], "off_color": [0, 0, 0],                   "animated": False, "frequency": 3, "duty_cycle": 0.5, 'use_mode': False}

    class Mode(Enum):
        """ Mode class is for showing robot's current scoring mode and is the default during teleop """
        kCORAL = {'name': "CORAL", "on_color": [200, 200, 200], "off_color": [0, 0, 0], "animated": False, "frequency": None, "duty_cycle": None}
        kALGAE = {'name': "ALGAE", "on_color": [0, 120, 120], "off_color": [0, 0, 0], "animated": False, "frequency": None, "duty_cycle": None}  # [0, 180, 180] still looks too blue
        kNONE = {'name': "NONE", "on_color": [160, 0, 160], "off_color": [0, 0, 0], "animated": False, "frequency": None, "duty_cycle": None}

    def __init__(self, robot_state: RobotState):
        super().__init__()
        self.setName('Led')
        self.robot_state = robot_state

        # Register LED to listen for RobotState updates
        self.robot_state.register_callback(self.update_from_robot_state)

        # try to start all the subsystems on a different count so they don't all do the periodic updates at the same time
        self.counter = constants.LedConstants.k_counter_offset
        self.animation_counter = 0
        # this should auto-update the lists for the dashboard.  you can iterate over enums
        self.indicators_dict = {indicator.value["name"]: indicator for indicator in self.Indicator}
        self.modes_dict = {mode.value["name"]: mode for mode in self.Mode}
        self.last_toggle_time = Timer.getFPGATimestamp()  # Tracks the last toggle time
        self.toggle_state = False  # Keeps track of the current on/off state

        # necessary initialization for the LED strip
        self.led_count = constants.LedConstants.k_led_count
        self.led_strip = AddressableLED(constants.LedConstants.k_led_pwm_port)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        self.set_leds((0, 0, 0))  # our own custom function

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        self._init_networktables()

        # initialize modes and indicators
        self.mode = self.Mode.kNONE
        self.prev_mode = self.Mode.kNONE
        self.indicator = Led.Indicator.kPOLKA

        self.set_mode(self.Mode.kNONE)
        self.set_indicator(self.Indicator.kNONE)

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.led_mode_pub = self.inst.getStringTopic(f"{constants.status_prefix}/_led_mode").publish()
        self.led_indicator_pub = self.inst.getStringTopic(f"{constants.status_prefix}/_led_indicator").publish()

    def update_from_robot_state(self, target, side):
        """ Update LED mode based on RobotState changes. """
        if target.value['mode'] == 'coral':
            self.set_mode(self.Mode.kCORAL)
        elif target.value['mode'] == 'algae':
            self.set_mode(self.Mode.kALGAE)
        elif target.value['mode'] == 'keep':
            pass  # don't change the mode
        else:
            self.set_mode(self.Mode.kNONE)
        self.set_indicator(self.Indicator.kNONE)

    def set_mode(self, mode) -> None:
        self.prev_mode = self.mode
        self.mode = mode
        self.led_mode_pub.set(self.mode.value['name'])

    def get_mode(self):
        return self.mode

    def set_indicator(self, indicator) -> None:
        self.indicator = indicator
        self.led_indicator_pub.set(self.indicator.value['name'])

    def set_indicator_with_timeout(self, indicator: Indicator, timeout: float) -> commands2.ParallelRaceGroup:
        return commands2.StartEndCommand(
            lambda: self.set_indicator(indicator),
            lambda: self.set_indicator(Led.Indicator.kNONE),
        ).withTimeout(timeout)

    def set_leds(self, color_or_data, start_index=0, end_index=None, is_hsv=False):
        """
        A helper function to efficiently set colors on a segment of the LED strip.
        This function can handle setting a single color to a range of LEDs or applying
        a list of colors (like an animation frame) to a range.

        :param color_or_data: Can be a single color tuple `(R, G, B)` or `(H, S, V)`,
                              or a list of such tuples `[(R,G,B), (R,G,B), ...]`.
        :param start_index: The starting LED index to apply the color(s) to (inclusive).
        :param end_index: The ending LED index (exclusive). If None, it goes to the end of the strip.
        :param is_hsv: A boolean flag. If True, `setHSV` is used. If False, `setRGB` is used.
        """
        # If no end_index is specified, default to the entire length of the strip.
        if end_index is None:
            end_index = self.led_count

        # Sanitize inputs to prevent IndexError. Ensures indices are within the valid range.
        start_index = max(0, start_index)
        end_index = min(self.led_count, end_index)

        # Determine if the input is a single color tuple or a list of color tuples (for animations).
        # This check is done by inspecting the type of the first element in the list.
        is_single_color = isinstance(color_or_data[0], (int, float))

        # --- Apply colors based on whether it's a single color or a list ---
        if is_single_color:
            # Optimization: Unpack the color tuple once before the loop to avoid repeated indexing.
            c1, c2, c3 = color_or_data
            # Set the same color for all LEDs in the specified range.
            for i in range(start_index, end_index):
                if is_hsv:
                    self.led_data[i].setHSV(c1, c2, c3)
                else:
                    self.led_data[i].setRGB(c1, c2, c3)
        else:
            # This branch handles a list of colors (e.g., an animation frame).
            data_len = len(color_or_data)
            # Iterate through the specified range of LEDs and apply the corresponding color from the list.
            for i in range(start_index, end_index):
                # The `i - start_index` maps the absolute LED index `i` to the
                # relative index in the `color_or_data` list.
                # This check prevents reading past the end of the color data list.
                if i - start_index < data_len:
                    color = color_or_data[i - start_index]
                    if is_hsv:
                        self.led_data[i].setHSV(*color)
                    else:
                        self.led_data[i].setRGB(*color)

    def periodic(self):
        self.counter += 1  # Increment the main counter
        if self.counter % 5 == 0:  # Execute every 5 cycles (10Hz update rate)
            current_time = Timer.getFPGATimestamp()
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

                    self.set_leds(color)

                else:  # Handle animated indicators
                    data = self.indicator.value["animation_data"]
                    if time_since_toggle > 1 / self.indicator.value["frequency"]:
                        self.animation_counter += 1
                        self.last_toggle_time = current_time

                    shift = self.animation_counter % self.led_count
                    shifted_data = data[shift:] + data[:shift]
                    self.set_leds(shifted_data, is_hsv=self.indicator.value["use_hsv"])

            else:  # Handle mode-only LEDs - they do not toggle
                # thinking of using the target state to light the robot
                lit_leds = self.robot_state.target.value['lit_leds']
                if lit_leds == constants.LedConstants.k_led_count:
                    self.set_leds(self.mode.value["on_color"])
                else:  # target dependent LED states:
                    # turn them all off
                    self.set_leds(self.mode.value["off_color"])
                    # turn on the first section
                    self.set_leds(self.mode.value["on_color"], end_index=lit_leds)
                    # turn on the other section
                    self.set_leds(self.mode.value["on_color"], start_index=self.led_count - lit_leds)

            self.led_strip.setData(self.led_data)  # Send LED updates

            if constants.LedConstants.k_nt_debugging:  # extra debugging info for NT
                pass
