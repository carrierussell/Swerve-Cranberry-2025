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
    public static String[] autoNames = {"Leave", "TrajectoryTest" };

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

    public static Command getSelectedAuto(String selectedAutoName, DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
    Command command = null;

   
    switch(selectedAutoName) {
        case "Leave":
        command = leave(robotDrive);                   // time based robot movement
        break;
        case "Leave Trajectory":                      //test to see if this moves the robot how we need it
        command = leaveTrajectory(robotDrive);
        break;
        case "Leave and Score":
        command = leaveAndScoreL3(robotDrive, coral, elevator);
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

    //score Coral in L2
    public static Command scoreL2Coral(Coral coral, Elevator elevator){
        return Commands.sequence(
            new RunCommand(() -> elevator.goToElevatorL2()),
            new RunCommand(() -> coral.scoreL24(), coral)
        );    
    }
    
    //score Coral in L3
    public static Command scoreCoralL3(Coral coral, Elevator elevator){
        return Commands.sequence(
             new RunCommand(() -> elevator.goToElevatorL3()), 
             new RunCommand(() -> coral.scoreL24())
        );
    }

        
    // Command to drive backwards for certain amount of time, hopefully to leave the starting zone in auto period.
    public static Command leave(DriveSubsystem robotDrive) {
        return Commands.sequence(
       startZoneMovement(robotDrive, 0.75 , 1.0)
    );
    }

    public static Command leaveAndScoreL3(DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
        return Commands.sequence(
           leave(robotDrive),
           scoreCoralL3(coral, elevator)
        );

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

    public static Command leaveTrajectory(DriveSubsystem robotDrive) {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
    
         // Generate trajectory (moves forward 2 meters)
        Trajectory leaveTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(2, 0, new Rotation2d(0)),
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
    

 
}