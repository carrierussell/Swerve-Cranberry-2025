package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.simulation.SimulatableCANSparkMax;

public class Climber extends SubsystemBase {
//public class Climber extends Subsystem{
  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Climber mClimber;
  private PeriodicIO mPeriodicIO;
  private SparkMax mMotor;
   
  public Climber() {
    super("Climber");

    mPeriodicIO = new PeriodicIO();

    //SparkMaxConfig climberConfig = new SparkMaxConfig();
    
    // CLIMBER MOTOR
    mMotor = new SparkMax(Constants.Climber.kClimberMotorId, MotorType.kBrushed);
  
     }

     public enum ClimberState {
        NONE,
        STOW,
        CAGE,
      }

  private static class PeriodicIO {
    double climber_target = 0.0;
    double climber_power = 0.0;
    boolean is_climber_pos_control = false;

    ClimberState state = ClimberState.STOW;

  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
   
  }

 public void stopMotor() {
    mPeriodicIO.is_climber_pos_control = false;
    mPeriodicIO.climber_power = 0.0;
    mMotor.set(0.0);
  }

 public ClimberState getState() {
    return mPeriodicIO.state;
  }

  public void setPower(double power) {
    mPeriodicIO.is_climber_pos_control = false;
    mPeriodicIO.climber_power = power;
  }

  public void climberUp(){
   mMotor.set(-0.75);
  }

  public void climberDown(){
   mMotor.set(0.75);
  }

  

}

  