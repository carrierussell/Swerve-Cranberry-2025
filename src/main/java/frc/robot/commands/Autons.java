package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.function.Supplier;



public final class Autons {
    /** Example static factory for an autonomous command. 
     public static Command exampleAuto(armSubsystem subsystem) {
    return Commands.sequence(subsystem.turnOn(1.0), new moveArmUp(subsystem));
    }
    */
    public static Supplier<Boolean> getLEDState;
    public static String[] autoNames = {"Leave", "Leave Trajectory", "Leave and ScoreL1" , "Leave and ScoreL2", "Leave and ScoreL3"};;

    // Initializes a DigitalInput on DIO 0 for the light break sensor
  //  static DigitalInput lightBeamSensor = new DigitalInput (1);

    // For field relative driving to work properly (in teleop/auton), the robot MUST be placed facing field forwards or backwards.
    // We can flip the gyro in code if necessary, but the orientation of the robot should be figured out first.


    private Autons() {
    throw new UnsupportedOperationException("This is a utility class!");
    }

    public interface BoolCallback {
      boolean execute();
    
};

    public static Command getSelectedAuto(String selectedAutoName, DriveSubsystem robotDrive, Coral coral, Elevator elevator, Algae algae) {
    Command command = null;

   
    switch(selectedAutoName) {
        case "Leave":
        command = leave(robotDrive);                   // time based robot movement
        break;
        case "Leave Trajectory":                      //test to see if this moves the robot how we need it
        command = leaveTrajectory(robotDrive);
        break;
        case "Leave and ScoreL1":
        command = leaveAndScoreL1(robotDrive, coral, elevator);
        break;
        case "Leave and ScoreL2":
        command = leaveAndScoreL2(robotDrive, coral, elevator);
        break;
        case "Leave and ScoreL3":
        command = leaveAndScoreL3(robotDrive, coral, elevator, algae);
        break;
        
    }

    return command;
    }

    public static Command startZoneMovement(DriveSubsystem robotDrive, double velocity, double distance) {
        double speed = velocity;
        double moveTime = distance;
        return Commands.sequence(
            new AutonSwerveControlCommand(robotDrive, speed, 0, 0, moveTime, false)
        );
    }

    //remove Algae from reef
    public static Command removeAlgae(DriveSubsystem robotDrive, Elevator elevator, Algae algae){
        //try seeing if we can get the Algae out of the reef during auton...
        return Commands.sequence(
            new RunCommand(() -> elevator.goToAlgaeLow(), elevator).withTimeout(Constants.Elevator.kElevatorMaxMoveTime),
            // OR
            new RunCommand(() -> elevator.goToAlgaeLow(), elevator).until(() -> elevator.hasReached()),
            moveAlgae(robotDrive),
            new RunCommand(() -> algae.grabAlgae(), algae).withTimeout(Constants.Algae.kAlgaeMaxTime),
            new RunCommand(() -> algae.stopAlgae(), algae).withTimeout(1.0), 
            new RunCommand(() -> elevator.goToElevatorStow(), elevator).withTimeout(Constants.Elevator.kElevatorMaxMoveTime)
        );
    }

    //score Coral in L2
    public static Command scoreL2Coral(Coral coral, Elevator elevator){
        // Govind: I added the ".withTimeout()" to these commands, because I think "RunCommand" will just run it forever without moving on to the next command.
        // For elevator movement, you could use ".until()" to make this more efficient, as long as elevator.hasReached() works (which is another function I added, see comments there).
        return Commands.sequence(
            new RunCommand(() -> elevator.goToElevatorL2(), elevator).withTimeout(Constants.Elevator.kElevatorMaxMoveTime),
            // OR
            new RunCommand(() -> elevator.goToElevatorL2(), elevator).until(() -> elevator.hasReached()),
            new RunCommand(() -> coral.scoreL24(), coral).withTimeout(Constants.Coral.kCoralScoreTime)
        );    
    }
    
    //score Coral in L3
    public static Command scoreCoralL3(Coral coral, Elevator elevator){
        return Commands.sequence(
    //might need to wait and intake the coral after the Algae is removed
            new RunCommand(() -> elevator.goToElevatorL3()).withTimeout(Constants.Elevator.kElevatorMaxMoveTime), 
            new RunCommand(() -> coral.scoreL24()).withTimeout(Constants.Coral.kCoralScoreTime),
            //new RunCommand(() -> coral.stopCoral()).withTimeout(1.0) //add this to stop coral after it scores
            Commands.runOnce(() -> coral.stopCoral(), coral), //add this to stop coral after it scores
            new RunCommand(() -> elevator.goToElevatorStow()).withTimeout(Constants.Elevator.kElevatorMaxMoveTime) // moves elevator back down to stow position after scoring
        );
    }

     //score Coral in L1()
     public static Command scoreCoralL1(Coral coral, Elevator elevator){
        return Commands.sequence(
            new RunCommand(() -> coral.scoreL24()).withTimeout(Constants.Coral.kCoralScoreTime),
            //new RunCommand(() -> coral.stopCoral()).withTimeout(1.0) //add this to stop coral after it scores
            Commands.runOnce(() -> coral.stopCoral(), coral) //add this to stop coral after it scores
        );
    }

        
    // Command to drive backwards for certain amount of time, hopefully to leave the starting zone in auto period.
    public static Command leave(DriveSubsystem robotDrive) {
        return Commands.sequence(
            startZoneMovement(robotDrive, 0.75 , 1.5)
        );
    }

    public static Command moveAlgae(DriveSubsystem robotDrive) {
        return Commands.sequence(
            startZoneMovement(robotDrive, 0.3 , 0.15)  //test this small move
        );
    }

    public static Command leaveAndScoreL1(DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
        return Commands.sequence(
            leaveTrajectory(robotDrive), // you could combine the driving in parallel with lifting the elevator, to really maximize time. See next command "leaveAndScoreL3_Parallel".
            scoreCoralL1(coral, elevator)
        );
    }

    public static Command leaveAndScoreL2(DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
        return Commands.sequence(
            // This parallel command will run "leave" and "goToElevatorL3" at the same time. 
            // goToElevatorL3 will only run once, which is all that is needed to set the target height for the elevator. The elevator will always move towards the target height.
            // The command will exit and move onto scoring after "leave" finishes (after robot stops driving).
            Commands.parallel(
                leaveTrajectory(robotDrive),
                Commands.runOnce(() -> elevator.goToElevatorL2(), elevator)
            ),
            scoreL2Coral(coral, elevator)
        );
    }

    public static Command leaveAndScoreL3(DriveSubsystem robotDrive, Coral coral, Elevator elevator, Algae algae) {
        return Commands.sequence(
            algaeTrajectory(robotDrive),
            removeAlgae(robotDrive, elevator, algae),   
            scoreCoralL3(coral, elevator)
        );
    }

    public static Command leaveTrajectory(DriveSubsystem robotDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
    
         // Generate trajectory (moves forward meters)
        Trajectory leaveTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.7, 0, new Rotation2d(0)),            //tested at 1.73 calculated to be 1.4 with back wheel on starting line
            config
        );
    
        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                leaveTrajectory,
                robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
    
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);
    
        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(leaveTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }

    public static Command algaeTrajectory(DriveSubsystem robotDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
    
         // Generate trajectory (moves forward meters)
        Trajectory leaveTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.38, 0, new Rotation2d(0)),            
            config
        );
    
        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                leaveTrajectory,
                robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
    
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);
    
        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(leaveTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }


    // Command to test swerve trajectory following in Auto
    public static Command testSimpleTrajectory(DriveSubsystem robotDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }

    

 
}