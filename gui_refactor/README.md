# Dashboard 2025 README (Draft)
### 20251015 CJH

## Step 0: PyQt6 Structure Overview

### Application Framework

* The dashboard is a **PyQt6 application** consisting of a single main window class `Ui`.
* All widgets are created as attributes of `Ui`, such as `self.qlabel_nt_connected`.

### UI Layout (.ui) & Designer workflow

* The visual layout lives in **`layout_2025.ui`** (Qt Designer format) and is loaded at runtime:

  ```python
  uic.loadUi('layout_2025.ui', self)
  ```

  Editing the `.ui` file updates the layout without changing Python code.
* Open/edit the file with **pyqt6-tools Designer** (or Qt Designer). Keep **objectName** values stable; the code uses them as keys in `widget_dict`.
* **Promoted/custom widgets used in the .ui:**

  * `QLabel2` → promoted from `QLabel` (header: `qlabel2.h`). These are the **clickable tiles** (e.g., `qlabel_wrist_up_indicator`, `qlabel_intake_on_indicator`).
  * `WarningLabel` → promoted from `QLabel` (header: `warning_label`). These are **numeric monitors** (e.g., `qlabel_pdh_voltage_monitor`, `qlabel_pdh_current_monitor`).
    Do not change their promoted class; the code relies on their behavior.
* **Adding a new tile/monitor** (Designer checklist):

  1. Place a `QLabel` and set a unique `objectName` using the existing naming patterns (`*indicator*`, `*monitor*`, etc.).
  2. If it should be clickable, **promote** it to `QLabel2` (base: `QLabel`, class name: `QLabel2`, header: `qlabel2.h`).
  3. If it should show a number with thresholds, **promote** to `WarningLabel` (base: `QLabel`, class name: `WarningLabel`, header: `warning_label`).
  4. Save the `.ui`. No recompilation step is needed because we call `uic.loadUi(...)` at startup.
  5. Add the new widget to `widget_dict` with its NT topics (see Step 2).

### Event Loop and Timers

* A `QTimer` drives periodic updates (`self.update_widgets()`), polling NetworkTables and refreshing the GUI.
* No GUI painting is done in background threads; all rendering occurs in the main thread.

### Signals and Slots

* Widgets emit Qt signals which connect to class methods or lambdas.
  Example:

  ```python
  combo.currentTextChanged.connect(self.update_routines)
  ```
* Some labels are **clickable**. A subclass emits `.clicked`, connected via:

  ```python
  label.clicked.connect(lambda key=label_key: self.label_click(key))
  ```

### Camera Thread

* A `QThread` runs a `CameraWorker(QObject)` that:

  * Opens an MJPEG or RTSP stream using OpenCV.
  * Converts each frame to a `QPixmap` using `convert_cv_qt`.
  * Emits finished/error signals when stopping or on failure.
* The GUI thread updates labels with `setPixmap()`.

### Styling and Custom Widgets

* Styling uses `.setStyleSheet(css_string)` with alternating color themes for blinks or state changes.
* `WarningLabel` is a custom class handling numeric display and color thresholding.

### Key Event Publishing

* The main window captures keypresses/releases and publishes pressed-key states back to NetworkTables.

---

## Step 1: NetworkTables / WPILib Overview

### NetworkTables Roles

* The robot runs the **NetworkTables (NT) server**.
* This dashboard runs as an **NT client**:

  ```python
  inst = ntcore.NetworkTableInstance.getDefault()
  inst.startClient4("PyQt Dashboard")
  inst.setServerTeam(2429)
  ```

### What the Robot Must Publish

* **Telemetry:** via `SmartDashboard.putNumber`, `putBoolean`, or `putString`.
* **Chooser (autonomous selector):** via `SendableChooser`.
* **Commands:** via `SmartDashboard.putData("Name", command)` — publishes a Command object to NT.

### Topic Categories

| Type         | Example                                       | Description                                                    |
| ------------ | --------------------------------------------- | -------------------------------------------------------------- |
| Boolean      | `/SmartDashboard/MoveWristUp/running`         | Created by `putData`; True schedules, False cancels a Command. |
| Number       | `/SmartDashboard/_pdh_voltage`                | Live numeric telemetry (voltage, angle, etc.).                 |
| String       | `/SmartDashboard/_target`                     | Textual state labels.                                          |
| Double Array | `/SmartDashboard/drive_pose`                  | Pose `[x, y, theta_deg]` arrays.                               |
| Chooser      | `/SmartDashboard/autonomous routines/options` | Produced automatically by `SendableChooser`.                   |

### Quick Robot-Side Examples

```python
# Basic telemetry
SmartDashboard.putNumber("_pdh_voltage", pdh.getVoltage())
SmartDashboard.putBoolean("gamepiece_present", sensor.get())

# Command exposed to dashboard
SmartDashboard.putData("MoveWristUp", MoveWristUpCommand())

# Autonomous chooser
autoChooser = SendableChooser()
autoChooser.setDefaultOption("DriveMiddle", "DriveMiddle")
autoChooser.addOption("ScoreL2", "ScoreL2")
SmartDashboard.putData("autonomous routines", autoChooser)
```

---

## Step 2: Widget Dictionary Architecture

All widgets and their NT mappings live in a central dictionary:

```python
self.widget_dict = {
    "<key>": {
        "widget": <Qt widget>,
        "nt": "<topic or None>",                 # Read topic for telemetry
        "command": "<cmd_running_topic or None>", # Command's /running boolean (from putData)
        "style_on": "<css>",                      # Optional ON style
        "style_off": "<css>",                     # Optional OFF style
        "flash": True/False,                       # Blink flag
        "selected": "<chooser_selected_topic>",   # Chooser selection topic
        "entry": <nt entry>,                       # NT entry for 'nt'
        "command_entry": <nt entry>                # NT entry for 'command'
    },
    ...
}
```

### Naming Conventions and Behaviors

| Key Pattern                | Behavior                         | Source / Action                                |
| -------------------------- | -------------------------------- | ---------------------------------------------- |
| `*indicator*`              | Boolean indicator                | Reads boolean from `nt`; updates color/style.  |
| `*monitor*`                | Numeric monitor (`WarningLabel`) | Reads double; auto-colors based on thresholds. |
| `*lcd*`                    | QLCDNumber numeric               | Reads double; shows as integer.                |
| `*combo*`                  | Chooser                          | Reads options array; writes selected string.   |
| `*time*`                   | Match timer                      | Reads seconds; styles during endgame.          |
| `drive_pose`, `quest_pose` | Pose arrays                      | Reads `[x, y, θ]`; updates field overlay.      |
| with `command`             | Command tile                     | Writes boolean to Command’s `/running` topic.  |

### Example Entries

```python
# Alliance color indicator
"qlabel_alliance_indicator": {
    "widget": self.qlabel_alliance_indicator,
    "nt": "/FMSInfo/IsRedAlliance",
    "command": None,
    "style_on":  "background-color:rgb(225,0,0); color:rgb(200,200,200);",
    "style_off": "background-color:rgb(0,0,225); color:rgb(200,200,200);",
},

# PDH voltage monitor
"qlabel_pdh_voltage_monitor": {
    "widget": self.qlabel_pdh_voltage_monitor,
    "nt": "/SmartDashboard/_pdh_voltage",
    "command": None
},

# Autonomous chooser
"qcombobox_autonomous_routines": {
    "widget": self.qcombobox_autonomous_routines,
    "nt": r"/SmartDashboard/autonomous routines/options",
    "selected": r"/SmartDashboard/autonomous routines/selected",
    "command": None
},

# Command tile
"qlabel_wrist_up_indicator": {
    "widget": self.qlabel_wrist_up_indicator,
    "nt": "/SmartDashboard/MoveWristUp/running",
    "command": "/SmartDashboard/MoveWristUp/running"
}
```

### Update Lifecycle

1. **Initialization:** Create NT entries, wire clickable labels, and connect chooser updates.
2. **Periodic Update:**

   * Indicators → Boolean → style toggle.
   * Monitors → Double → value + WarningLabel color.
   * Combos → refresh options, sync selected.
   * Time/Pose → specialized update.
3. **User Actions:** Click command → toggles `command_entry`. Combo select → updates chooser’s `selected` topic.

---

## Step 3: Advanced Logic (Pose, QuestNav, Distance)

### Field Pose Rendering

* Subscribes to `/SmartDashboard/drive_pose` and `/SmartDashboard/quest_pose`.
* Each provides `[x, y, heading_deg]` in field coordinates.
* The GUI draws robot markers scaled to the field image size.
* Heading convention: +θ = counterclockwise degrees.

### QuestNav Pose Fusion

* Topics used:

  * `/SmartDashboard/questnav_in_use` (boolean)
  * `/SmartDashboard/questnav_synched` (boolean)
  * Command toggles: `/SmartDashboard/QuestEnableToggle/running`, `/SmartDashboard/QuestSyncToggle/running`, `/SmartDashboard/QuestResetOdometry/running`
* Indicators:

  * **Enabled/In Use** (`qlabel_questnav_enabled_toggle_indicator`): reads `questnav_in_use` and writes `QuestEnableToggle/running` on click.
  * **Synced** (`qlabel_questnav_sync_toggle_indicator`): reads `questnav_synched` and writes `QuestSyncToggle/running` on click.
* Colors/styles:

  * By default, indicators use the global **ON = green**, **OFF = red** styles from `update_widgets()`.
  * If you uncomment the per-widget `style_on/style_off` overrides in `widget_dict`, the **Enabled** tile becomes **cyan when true** and **grey when false** (as shown in the commented CSS). There is **no yellow state** in the current code.
* Behavior note: these are **independent booleans**; there isn’t a tri‑state (grey/yellow/green) logic unless you add it via per-widget overrides.

### Shot Distance Calculation

* Combines robot `drive_pose` and target pose (from `/SmartDashboard/_target` or constants).
* Computes Euclidean distance and launch angle.
* Displays live numeric values and warning color if outside calibrated range.

---

### Summary

This dashboard provides a general-purpose PyQt6 GUI for FRC robots using NetworkTables v4.
It reads telemetry, allows command activation via `putData` commands, supports chooser selection, and renders live field poses and status indicators in real time.
