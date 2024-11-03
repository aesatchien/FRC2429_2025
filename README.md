## FRC2429_2025

### Code repo for 2025 using robotpy
FRC robot code for the 2025 robot using robotpy-commands-v2.  Note, there has been no attempt to account for 2025 breaking changes as of Nov 3, 2025.

* Note code will be updated often during Jan-April 2025 build season

#### Organization
* robot - 2025 robot code - TBD
* gui  - 2025 version of the python dashboard(s)
* resources
   * sim  - utilities for the simulation (field images, etc)
* tests - simple one-file robots for testing a system using wpilib.TimedRobot
  * distance sensors, drivetrain, elevator/turret/wrist/arm, pneumatics
* other_robots  - code for practice bots, characterization, etc
   * swerve_base - stripped-down swerve base
   * wcd_bot - stripped-down WCD base


---
#### Where's the other stuff?
* training notebooks are here: https://github.com/aesatchien/FRC_training
* vision system (image processing on the pi) is here:  https://github.com/aesatchien/FRC2429_vision

#### Installation
Clone the git and install on your own machine:
Use "git clone https://github.com/aesatchien/FRC2429_2025.git" from the git bash (or any git aware) shell to download.  If you don't have git and you just want to look at the code, you can download the repository from the links on the right.

Notes on how to install python and the necessary accessories (particularly the robotpy libraries) that will get all of this running:
[2025 Python Installation](https://docs.google.com/document/d/17hmov_FAXwbJdad-m56ibuYFLzUg6PLvJjhuGWcnYeQ/edit?usp=sharing) but may be a bit cryptic.  I'll help if you need it.
basically, in your python environment (you may want more than one) you'll need a `pip install robotpy` and if you are working on anything else several more packages.

Once everything is installed, you need to go to the folder with robot.py.  From there, commands like (new for 2024)

```python -m robotpy sim```  

should bring up the simulator and allow you to check to see if your gamepad is recognized (you can also use the keyboard) and should be able to let you drive a virtual robot around the field if you have a gamepad. 
```python -m robotpy deploy``` and ```python -m robotpy sync``` will send your code to the robot and sync your packages, respectively.
[Read more about it here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html).
