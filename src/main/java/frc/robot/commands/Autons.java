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
import edu.wpi.first.math.util.Units;
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
    public static String[] autoNames = {"Leave", "Leave Trajectory", "Leave and ScoreL1" , "Leave and ScoreL2", "Leave and ScoreL3", "Leave Side and ScoreL1", "3 Coral Auton"};;

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
        command = AutoTrajectory.leaveTrajectory(robotDrive);
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
        case "Leave Side and ScoreL1":
        command = leaveAndScoreSideL1(robotDrive, coral, elevator);
        break;
        case "3 Coral Auton":
        command = leaveAndScoreL2Reload(robotDrive, coral, elevator);
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
            AutoTrajectory.leaveTrajectory(robotDrive), // you could combine the driving in parallel with lifting the elevator, to really maximize time. See next command "leaveAndScoreL3_Parallel".
            scoreCoralL1(coral)
        );
    }

    public static Command leaveAndScoreSideL1(DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
        return Commands.sequence(
            AutoTrajectory.leaveSideTrajectory(robotDrive), // you could combine the driving in parallel with lifting the elevator, to really maximize time. See next command "leaveAndScoreL3_Parallel".
            scoreCoralL1(coral)
        );
    }

    public static Command leaveAndScoreL2(DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
        return Commands.sequence(
            // This parallel command will run "leave" and "goToElevatorL3" at the same time. 
            // goToElevatorL3 will only run once, which is all that is needed to set the target height for the elevator. The elevator will always move towards the target height.
            // The command will exit and move onto scoring after "leave" finishes (after robot stops driving).
            Commands.parallel(
                AutoTrajectory.L2SimpleTrajectory(robotDrive),
                Commands.runOnce(() -> elevator.goToElevatorL2(), elevator)
            ),
            scoreL2Coral(coral, elevator)
        );
    }

    public static Command leaveAndScoreL2Reload(DriveSubsystem robotDrive, Coral coral, Elevator elevator) {
        return Commands.sequence(
            // This parallel command will run "leave" and "goToElevatorL2" at the same time. 
            // goToElevatorL2 will only run once, which is all that is needed to set the target height for the elevator. The elevator will always move towards the target height.
            // The command will exit and move onto scoring after "leave" finishes (after robot stops driving).
            Commands.parallel(
                AutoTrajectory.L2SimpleTrajectory(robotDrive),
                Commands.runOnce(() -> elevator.goToElevatorL2(), elevator)
            ),
            scoreL2Coral(coral, elevator),
            AutoTrajectory.L2ToPlayerTrajectory(robotDrive),
            startZoneMovement(robotDrive, 0.0 , 1.5),
            AutoTrajectory.PlayerToL2Trajectory(robotDrive),
            scoreCoralL1(coral),
            //scoreL2Coral(coral,elevator)   //If lined up with reef, move from L1 score to L2 score
            AutoTrajectory.L2FrontReturnPlayerTrajectory(robotDrive)
            //startZoneMovement(robotDrive, 0.0 , 1.5),
            //scoreCoralL1(coral)

        );
    }

    public static Command leaveAndScoreL3(DriveSubsystem robotDrive, Coral coral, Elevator elevator, Algae algae) {
        return Commands.sequence(
            AutoTrajectory.algaeTrajectory(robotDrive),
            removeAlgae(robotDrive, elevator, algae),   
            scoreCoralL3(coral, elevator)
        );
    }

    //score Coral in L1()
    public static Command scoreCoralL1(Coral coral){
    return Commands.sequence(
        new RunCommand(() -> coral.scoreL1()).withTimeout(Constants.Coral.kCoralScoreTime),
        //new RunCommand(() -> coral.stopCoral()).withTimeout(1.0) //add this to stop coral after it scores
        Commands.runOnce(() -> coral.stopCoral(), coral) //add this to stop coral after it scores
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
            new RunCommand(() -> coral.scoreL23(), coral).withTimeout(Constants.Coral.kCoralScoreTime),
            new RunCommand(() -> coral.stopCoral(), coral).withTimeout(1.0)
        );    
    }
    
    //score Coral in L3
    public static Command scoreCoralL3(Coral coral, Elevator elevator){
        return Commands.sequence(
    //might need to wait and intake the coral after the Algae is removed
            new RunCommand(() -> elevator.goToElevatorL3()).withTimeout(Constants.Elevator.kElevatorMaxMoveTime), 
            new RunCommand(() -> coral.scoreL23()).withTimeout(Constants.Coral.kCoralScoreTime),
            //new RunCommand(() -> coral.stopCoral()).withTimeout(1.0) //add this to stop coral after it scores
            Commands.runOnce(() -> coral.stopCoral(), coral), //add this to stop coral after it scores
            new RunCommand(() -> elevator.goToElevatorStow()).withTimeout(Constants.Elevator.kElevatorMaxMoveTime) // moves elevator back down to stow position after scoring
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
            Commands.parallel(
            new RunCommand(() -> algae.stopAlgae(), algae).withTimeout(3.0),
            startZoneMovement(robotDrive, -0.5,0.5),
            new RunCommand(() -> elevator.goToElevatorStow(), elevator).withTimeout(Constants.Elevator.kElevatorMaxMoveTime))
        );
    }



  
 
}