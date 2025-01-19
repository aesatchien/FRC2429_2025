# import math
# import commands2
# from wpilib import SmartDashboard
# from subsystems.lower_crank import LowerCrank


# class MoveLowerArmByNetworkTables(commands2.Command):

#     def __init__(self, container, crank: LowerCrank) -> None:
#         super().__init__()
#         self.setName('Move lower arm by networktables')  # change this to something appropriate for this command
#         self.container = container
#         self.crank = crank

#         SmartDashboard.putNumber("Lower crank setpoint degrees", self.crank.get_angle())

#         self.addRequirements(self.crank)  # commandsv2 version of requirements

#     def initialize(self) -> None:
#         """Called just before this Command runs the first time."""
#         self.start_time = round(self.container.get_enabled_time(), 2)
#         print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
#         SmartDashboard.putString("alert",
#                                  f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

#         self.crank.set_position(math.radians(
#             SmartDashboard.getNumber("Lower crank setpoint degrees", 
#                     math.degrees(self.crank.get_angle())
#             )
#         ))

#     def execute(self) -> None:
#         pass

#     def isFinished(self) -> bool:
#         return True

#     def end(self, interrupted: bool) -> None:
#         end_time = self.container.get_enabled_time()
#         message = 'Interrupted' if interrupted else 'Ended'
#         print_end_message = False
#         if print_end_message:
#             print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
#             SmartDashboard.putString(f"alert",
#                                      f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
