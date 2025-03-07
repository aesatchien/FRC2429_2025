class PathPlannerConfiguration():

    def __init__(self) -> None:
        pass

    # This is a method that will configure the paths for the robot to follow, based on the .path files in the deploy/pathplanner/paths directory.
    def configure_paths(self, autonomous_chooser:wpilib.SendableChooser):
        allowed_autos = ['1+1 amp', '1+1 middle', '1+1 source', '1+0', "1+0 leave amp", "1+0 leave source", "1+1 amp with midline", "1+1 source with midline", "1+2 middle source", "1+2 middle amp", "1+1 and leave source side AVR", "1+3"]
        only_use_allowed_autos = True # Change to false for testing, true for comp.  Does not allow paths, only autos, when true.


        if wpilib.RobotBase.isReal():
            path_to_pathplanner_trajectories = '/home/lvuser/py/deploy/pathplanner/paths'
            path_to_pathplanner_autos = '/home/lvuser/py/deploy/pathplanner/autos'
        else:
            path_to_pathplanner_trajectories = os.path.abspath(constants.k_path_from_robot_to_pathplanner_files)
            path_to_pathplanner_autos = os.path.abspath(constants.k_path_from_robot_to_pathplanner_autos)

        file_names = os.listdir(path_to_pathplanner_trajectories) + os.listdir(path_to_pathplanner_autos)

        # No more setting default command here because default command should be drivewait which is not pathplanner auto 3/21/24 LHACK
        for file_name in file_names:
            pure_name = os.path.splitext(file_name)[0]
            extension = os.path.splitext(file_name)[1]
            # We don't want paths, only autos
            if extension == '.auto':
                if only_use_allowed_autos and pure_name in allowed_autos:
                    autonomous_chooser.addOption(pure_name, PathPlannerAuto(pure_name))

                elif not only_use_allowed_autos:
                    print(f'Adding {pure_name} since we\'re adding all autos')

            elif extension == '.path' and not only_use_allowed_autos:
                autonomous_chooser.addOption(pure_name, AutoBuilder.followPath(PathPlannerPath.fromPathFile(pure_name)))

