# FRC Dashboard 2025 (Refactored)

This document describes the architecture and workflow for the 2025 FRC PyQt6 Dashboard. The project has been refactored from a single-file application into a modular structure to improve maintainability and clarity.

## Project Structure

The application is now broken into several specialized Python modules:

-   `main.py`: The application entry point. It initializes the QApplication and the main `Ui` window.
-   `dashboard_2025.py`: Defines the main window class `Ui`. It orchestrates the other manager modules.
-   `config.py`: **Central configuration file.** Contains the static `WIDGET_CONFIG` and `CAMERA_CONFIG` dictionaries that define all widget properties, NetworkTables topics, and camera streams.
-   `nt_manager.py`: Handles NetworkTables (NT) connection logic, server switching, and connection reporting.
-   `camera_manager.py`: Manages the camera stream, including the worker thread for fetching and decoding video frames.
-   `ui_updater.py`: Contains the core logic for updating all GUI elements. It uses NT4 Subscribers to poll data and implements optimization logic to only repaint widgets when values change.
-   `nt_tree.py`: Manages the UI and logic for the interactive NetworkTables tree viewer.
-   `widgets/`: A directory containing custom PyQt widget classes like `ClickableQLabel` and `WarningLabel`.
-   `layout_2025.ui`: The Qt Designer UI file that defines the visual layout of the dashboard.

---

## Step 1: How to Add or Modify a Widget

The workflow for adding a new widget (e.g., an indicator tile or a numeric display) has been streamlined.

### 1. Update the UI Layout

-   Open `layout_2025.ui` in **Qt Designer** (or `pyqt6-tools designer`).
-   Add a new widget (e.g., a `QLabel`).
-   In the **Property Editor**, give it a unique and descriptive `objectName` (e.g., `qlabel_my_new_indicator`).
-   If the widget should be clickable or have custom behavior, promote it to the appropriate class from the `widgets/` directory (e.g., `ClickableQLabel`).
-   Save the `.ui` file. No code generation or compilation is needed.

### 2. Update the Configuration

-   Open `config.py`.
-   **Shortcut for Commands:** If you are adding a standard command indicator, simply add the command name to the `COMMAND_LIST` list in `config.py`. The configuration will be auto-generated.
-   Add a new entry to the `WIDGET_CONFIG` dictionary. The key should be a descriptive name, and the value is a dictionary defining its properties.

**Example: Adding a new command indicator tile:**

```python
# In config.py, inside WIDGET_CONFIG
'my_new_indicator': {
    'widget_name': 'qlabel_my_new_indicator',      # The objectName from Qt Designer
    'nt_topic': '/SmartDashboard/MyCommand/running', # The NT topic it reads from
    'command_topic': '/SmartDashboard/MyCommand/running' # The NT topic to write to on click
},
```

The application will automatically find the widget by its name and bind it to the specified NetworkTables topics at startup.

---

## Step 2: Architectural Overview

### Main Window (`dashboard_2025.py`)

The `Ui` class in `dashboard_2025.py` is the central coordinator. Its responsibilities are:
-   Loading the `.ui` file.
-   Initializing the manager classes (`NTManager`, `CameraManager`, `UIUpdater`, `NTTreeManager`).
-   Building the runtime `widget_dict` and `camera_dict` from the static configurations in `config.py`.
-   Connecting UI signals (button clicks, etc.) to methods in the appropriate managers or in the `Ui` class itself.
-   Managing key press/release events.

### Configuration (`config.py`)

This file decouples the widget and camera definitions from the application logic.
-   `WIDGET_CONFIG`: Defines UI widgets. Each entry maps a widget's `objectName` to its NetworkTables topics, custom styles, and behavior flags.
-   `CAMERA_CONFIG`: Defines the available camera streams and their associated NT topics for status monitoring.

### NetworkTables Manager (`nt_manager.py`)

-   Manages the `NetworkTableInstance`.
-   Handles connecting to the robot, server switching (sim vs. robot), and reconnecting.
-   *Note: Direct data access is now handled via `ntcore` Subscribers in `dashboard_2025.py` and `ui_updater.py`.*

### UI Updater (`ui_updater.py`)

-   The `update_widgets()` method is called by a `QTimer` on the main thread.
-   It iterates through the `widget_dict` and updates each widget's appearance (color, text, value) based on the latest data received from NetworkTables.
-   This module contains all the styling logic (e.g., `style_on`, `style_off`, flashing) that was previously in the main class.
-   **Optimization:** This class implements a "dirty bit" check (`last_value`). It compares the current NT value with the previous update's value and only triggers expensive Qt repaints (`setStyleSheet`, `setText`) if the data has actually changed.

### Camera Manager (`camera_manager.py`)

-   Manages the `QThread` and `CameraWorker` for non-blocking video streaming.
-   Handles starting, stopping, and restarting the camera thread.
-   Checks for the availability of camera servers before starting the stream.

---

## Step 3: NetworkTables and Robot-Side Code

The interaction with NetworkTables remains the same from the robot's perspective. The dashboard client expects the robot server to publish topics for telemetry and commands.

### NT4 Architecture (Publisher/Subscriber)
The dashboard uses the modern NT4 **Publisher/Subscriber** model provided by the `ntcore` library:
-   **Subscribers:** Used for reading data (Telemetry, Pose, Status). They are read-only and efficient.
-   **Publishers:** Used for sending data (Commands, Chooser selections).

### Topic Categories

| Type | Example | Description |
|---|---|---|
| **Command** | `/SmartDashboard/MoveWristUp/running` | A boolean that the dashboard sets to `true` to run a command. Published by `SmartDashboard.putData()`. |
| **Telemetry** | `/SmartDashboard/_pdh_voltage` | A number, boolean, or string representing robot state. Published by `SmartDashboard.putNumber()`, etc. |
| **Chooser** | `/SmartDashboard/autonomous routines/options` | A string array for the autonomous selector. Published by `SendableChooser`. |
| **Pose** | `/SmartDashboard/drive_pose` | A `double[]` of `[x, y, theta_deg]` for field visualization. |

### Robot-Side Example (Java/Python)

```java
// Exposing a command to the dashboard
SmartDashboard.putData("MoveWristUp", MoveWristUpCommand());

// Publishing telemetry
SmartDashboard.putNumber("_pdh_voltage", pdh.getVoltage());
SmartDashboard.putBoolean("gamepiece_present", sensor.get());
```
