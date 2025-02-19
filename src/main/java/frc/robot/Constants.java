package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 16;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }
  
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

    public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class Climber{
    public static final int kClimberMotorId = 30;
  }

  public static class Elevator {
    public static final int kElevatorLeftMotorId = 8;
    public static final int kElevatorRightMotorId = 9;

    public static final double kP = 0.15;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kIZone = 5.0;
    public static final double kG = 0.5;

    public static final double kMaxVelocity = 65;
    public static final double kMaxAcceleration = 200;

    public static final int kMaxCurrent = 40;

    public static final double kStowHeight = 0.0;
    public static final double kL2Height = 9.0;
    public static final double kL3Height = 25.14;
    public static final double kL4Height = 52.0;
    public static final double kMaxHeight = 56.2;
    public static final double kGroundAlgaeHeight = 0.0;
    public static final double kScoreAlgaeHeight = 0.0;
    public static final double kLowAlgaeHeight = 24.8;
    public static final double kHighAlgaeHeight = 42.5;

    public static final double kSetpointThreshold = 2.0; // Acceptable distance in encoder units between the elevator's actual position and its current target height.

    // Auto constants for Elevator
    public static final double kElevatorMaxMoveTime = 2.0; // Time it takes for the elevator to move from StowHeight to MaxHeight
  }

  public static class Coral {
    public static final int kLeftMotorId = 4;  //updated to match our architecture
    public static final int kRightMotorId = 5; //uodated to match our architecture

    public static final int kLaserId = 33;  //updated to match our architecture
    public static final int kColorId = 32;  //updated to match our architecture but this isn't used

    public static final double kMaxCurrent = 30;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIZone = 0;

    public static final double kIntakeSpeed = 0.4;
    public static final double kReverseSpeed = -0.4;
    public static final double kL1Speed = 0.5;
    public static final double kL24Speed = 0.5;
    public static final double kIndexSpeed = 0.1;
    public static final double kSpeedDifference = kL1Speed * 0.5;

    // Auto constants for Coral
    public static final double kCoralScoreTime = 2.0; // Time it takes for the coral to be scored.
  }

  public static class Algae {
    // WRIST
    public static final int kWristMotorId = 7;       //updated to match our architecture
    public static final int kIntakeMotorId = 6;     //updated to match our architecture

    public static final int kWristEncoderId = 9;     //this is a DIO input for the through bore Encoder
    public static final int kMaxWristCurrent = 10;  

    public static final double kWristP = 0.01;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.0;
    public static final double kWristKG = 0.0;
    public static final double kWristKV = 0.100;
    public static final double kWristKA = 0.0;

    public static final double kWristOffset = 141.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    public static final double kStowAngle = 233.0;
    public static final double kDeAlgaeAngle = 250.0;
    public static final double kGroundIntakeAngle = 330.0;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kIntakeSpeed = 0.8;
    public static final double kEjectSpeed = -0.5;
    public static final double kGroundIntakeSpeed = -0.5;
  }

  
  
// test this
  public static class Field {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    public static final Pose2d redCenterPose2d = new Pose2d(15.19, 5.50, new Rotation2d(Units.degreesToRadians(180.0)));
    public static final Pose2d blueCenterPose2d = new Pose2d(1.27, 5.50, new Rotation2d(0));
  }

  public static class LEDs {
    public static final int k_PWMId = 9;
    public static final int k_totalLength = 300;
  }
}
