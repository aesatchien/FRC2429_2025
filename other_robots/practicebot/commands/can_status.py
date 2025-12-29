import random

import commands2
import wpilib
from wpilib import SmartDashboard
import ntcore
import rev
import pickle as pkl
import random

from helpers.log_command import log_command
from subsystems.swerve_constants import DriveConstants as dc

@log_command(console=True, nt=False, print_init=True, print_end=False)
class CANStatus(commands2.Command):  # change the name for your command

    def __init__(self, container, ) -> None:
        super().__init__()
        self.setName('CANStatusCheck')
        self.container = container
        #self.addRequirements()  # commandsv2 version of requirements

        # Corrected swerve module indexing for RF (module 1) and LB (module 2)
        # TODO - see if i can't get this from driveconstants w/o hand-coding it
        self.can_ids = {#2: {'name': 'climber', 'motor': self.container.climber.sparkmax},
                        #3: {'name': 'climber', 'motor': self.container.climber.follower},
                        #4: {'name': 'elevator', 'motor': self.container.elevator.motor},
                        #5: {'name': 'elevator', 'motor': self.container.elevator.follower},
                        #6: {'name': 'shoulder', 'motor': self.container.pivot.motor},
                        #7: {'name': 'shoulder', 'motor': self.container.pivot.follower},
                        #10: {'name': 'wrist', 'motor': self.container.wrist.sparkmax},
                        #12: {'name': 'intake', 'motor': self.container.intake.spark_flex},
                        20: {'name': 'lf_turn',  'motor': self.container.swerve.swerve_modules[0].turningSpark},
                        22: {'name': 'lb_turn', 'motor': self.container.swerve.swerve_modules[2].turningSpark},
                        24: {'name': 'rf_turn', 'motor': self.container.swerve.swerve_modules[1].turningSpark},
                        26: {'name': 'rb_turn', 'motor': self.container.swerve.swerve_modules[3].turningSpark},
                        21: {'name': 'lf_drive', 'motor': self.container.swerve.swerve_modules[0].drivingSpark},
                        23: {'name': 'lb_drive', 'motor': self.container.swerve.swerve_modules[2].drivingSpark},
                        25: {'name': 'rf_drive', 'motor': self.container.swerve.swerve_modules[1].drivingSpark},
                        27: {'name': 'rb_drive', 'motor': self.container.swerve.swerve_modules[3].drivingSpark}
                        }
        self.fault_ids = {0:'kBrownout', 1:'kOvercurrent', 2:'kIWDTReset', 3:'kMotorFault', 4:'kSensorFault',
                          5:'kStall', 6: 'kEEPROMCRC', 7: 'kCANTX', 8: 'kCANRX', 9: 'kHasReset',
                          10: 'kDRVFault', 11: 'kOtherFault', 12: 'kSoftLimitFwd', 13: 'kSoftLimitRev',
                            14:'kHardLimitFwd', 15:'kHardLimitRev'}

        self.write_log = False

        # Set up NT4 publishers for efficiency and organization
        #self.inst = ntcore.NetworkTableInstance.getDefault()
        #self.can_status_pubs = {key: self.inst.getStringTopic(f"/SmartDashboard/CAN/CANID {key:02d}").publish()
        #                       for key in self.can_ids.keys()}

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        pass  # put in execute so log_command can send the time

        # if self.write_log:
        #     try:
        #         with open('can.pkl', 'rb') as file:
        #             records = pkl.load(file)
        #     except FileNotFoundError:  # start with a header row
        #         header = [f"{key} {self.can_ids[key]['name']}" for key in self.can_ids.keys()]
        #         records = [header]
        #
        #     output = {}
        #     for key in self.can_ids: # can't pickle a motor
        #         output.update({key:{'NAME': self.can_ids[key]['name'], 'STICKY_FAULTS': self.can_ids[key]['sticky_faults'],
        #                             'SET_BITS': self.can_ids[key]['set_bits'], 'FAULT_CODES': self.can_ids[key]['fault_codes']}})
        #
        #     just_faults = [self.can_ids[key]['fault_codes'] for key in self.can_ids.keys()]
        #
        #     records.append(just_faults)
        #
        #     with open('can.pkl', 'wb') as file:
        #         pkl.dump(records, file)
        #     print('Wrote CAN faults to can.pkl')
        #
        #     to read, just do this script
        #     import pickle as pkl
        #     import pandas as pd
        #     with open('can.pkl', 'rb') as file:
        #         data = pkl.load(file)
        #     column_names = data[0]
        #     df = pd.DataFrame(data[1:], columns=column_names)
        #     df

    def execute(self) -> None:
        # single execution and end
        for key in self.can_ids.keys():
            motor: rev.SparkBase = self.can_ids[key]['motor']
            sticky_faults_mask = 0

            # In the 2025 REV library, getStickyFaults() returns an object.
            # We must use .rawBits to get the integer bitmask.
            if wpilib.RobotBase.isSimulation():
                sticky_faults_mask = random.randint(0, 2048) # Simulate random faults
            else:
                sticky_faults_mask = motor.getStickyFaults().rawBits

            motor.clearFaults()  # This clears active (non-sticky) faults.

            set_bits = []
            binary_string = bin(sticky_faults_mask)[2:]  # convert to binary and ignore the initial two 0b characters
            for i, bit in enumerate(reversed(binary_string), start=0):
                # Check if the bit is set (i.e., equals '1')
                if bit == '1':
                    # Add the position (0-based indexing) of the set bit to the list
                    set_bits.append(i)

            fault_codes = [self.fault_ids[id] for id in set_bits if id in self.fault_ids]
            self.can_ids[key].update({'sticky_faults': sticky_faults_mask})
            self.can_ids[key].update({'set_bits': set_bits})
            self.can_ids[key].update({'fault_codes': fault_codes})

            status_string = f"{self.can_ids[key]['name']:13} sticky_faults: {sticky_faults_mask} {set_bits} {fault_codes}"
            print(f"CANID {key:02d}: {status_string}")
            # self.can_status_pubs[key].set(status_string)

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
