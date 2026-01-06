# LED Subsystem Primer

This document explains how the `Led` subsystem works. The goal of this subsystem is to provide clear, visual feedback to the drivers and alliance partners about the robot's current state and actions.

## 1. WPILib `AddressableLED` Basics

At its core, the WPILib `AddressableLED` class treats an LED strip as an array of `LEDData` objects. The process is simple:

1.  **Create an Array:** You create a list of `LEDData` objects, one for each pixel on your strip.
2.  **Set Colors:** You loop through this list and set the color of each pixel individually using `.setRGB(r, g, b)` or `.setHSV(h, s, v)`.
3.  **Send Data:** You call `led_strip.setData(your_led_data_list)` once per loop to send all the color information to the physical LED strip.

Our `led.py` builds a versatile abstraction on top of this simple model.

---

## 2. Our Architecture: Modes vs. Indicators

To manage complexity, we've separated LED states into two distinct categories:

### Modes (`Led.Mode`)
-   **Purpose:** Represents the robot's **persistent scoring state**. This is the "background" pattern.
-   **Examples:** `kCORAL`, `kALGAE`.
-   **Behavior:** When no special indicator is active, the LEDs will show the current `Mode`. The pattern can change based on the scoring `Target` set in `RobotState` (e.g., lighting up a different number of LEDs for L1 vs. L4).

### Indicators (`Led.Indicator`)
-   **Purpose:** Represents a **temporary, high-priority event** or a special animation.
-   **Examples:** `kSUCCESSFLASH`, `kFAILUREFLASH`, `kRAINBOW`.
-   **Behavior:** When an `Indicator` is active, it **overrides** the current `Mode` display. It can be a flashing color or a continuous animation. Indicators are often set with a timeout, after which the LEDs revert to displaying the current `Mode`.

---

## 3. How It Works: The `periodic` Loop

The `periodic` method in `led.py` runs every 100ms (`if self.counter % 5 == 0`) and makes a decision based on the current state.

1.  **Is an Indicator active?**
    -   If `self.indicator` is anything other than `kNONE`, the subsystem focuses on displaying the indicator.
    -   **Animated? (`kRAINBOW`, `kPOLKA`):** If the indicator is animated, it uses an `animation_counter` to shift the colors in the `animation_data` list, creating a moving pattern.
    -   **Flashing? (`kSUCCESS`, `kFAILURE`):** If the indicator is a flashing color, it uses a `Timer` and a `toggle_state` boolean to alternate between the indicator's `on_color` and its `off_color`.
        -   A special key, `use_mode: True`, tells a flashing indicator to use the current `Mode` color as its "off" color, creating a "Success + Mode" flash effect.

2.  **If no Indicator is active:**
    -   The subsystem defaults to displaying the current `Mode` (e.g., `kCORAL`).
    -   It reads the `lit_leds` value from the current `RobotState.Target`.
    -   It then turns on a specific number of LEDs from both ends of the strip to create a "filling up" effect that corresponds to the scoring height.

---

## 4. Key Code Features Explained

### The `Enum` Dictionaries
Both `Led.Mode` and `Led.Indicator` are `Enum` classes where each member is a dictionary that defines its behavior. This makes the configuration for each pattern clean and self-contained.

```python
# A flashing indicator
kSUCCESSFLASH = {
    'name': "SUCCESS + MODE",
    "on_color":,
    "off_color":, # Not used if use_mode is True
    "animated": False,
    "frequency": 3,         # Flashes 3 times per second
    "duty_cycle": 0.5,      # On for 50% of the time
    'use_mode': True        # Use the current Mode color for the "off" state
}

# An animated indicator
kRAINBOW = {
    'name': "RAINBOW",
    "animated": True,
    "frequency": 5,         # How fast the animation shifts
    'animation_data': [(...)] # A pre-calculated list of HSV colors
}
```

### `set_indicator_with_timeout()`
This is a **command factory** method. It's a convenient way to create a command that shows an indicator for a specific duration.

**Usage in `robotcontainer.py`:**
```python
# When this command is scheduled, it sets the indicator to SUCCESSFLASH.
# After 2 seconds, the command automatically ends, and the end() part
# sets the indicator back to kNONE.
commands2.CommandScheduler.getInstance().schedule(
    self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))
```

### `set_leds()` Helper Function
This is a utility function inside the `Led` class. Instead of manually looping through the `led_data` array every time, we can just call this function. It's smart enough to handle:
-   Setting a single color to the whole strip.
-   Setting a single color to just a *segment* of the strip (using `start_index` and `end_index`).
-   Applying a list of colors (an animation frame) to the strip.

### Connection to `RobotState`
The `Led` subsystem doesn't decide on its own what `Mode` to be in. It **subscribes** to updates from the `RobotState` subsystem.

1.  In `led.py`, `self.robot_state.register_callback(self.update_from_robot_state)` tells `RobotState` to call the `update_from_robot_state` method whenever the state changes.
2.  When the driver selects a new scoring target (e.g., `L4`), `RobotState` updates its internal state.
3.  `RobotState` then calls the `update_from_robot_state` method in the `Led` subsystem.
4.  This method checks the new target's `mode` (e.g., 'coral') and calls `self.set_mode(self.Mode.kCORAL)`.

This **callback** pattern decouples the two subsystems. `Led` doesn't need to know *why* the mode changed, only that it *did* change.