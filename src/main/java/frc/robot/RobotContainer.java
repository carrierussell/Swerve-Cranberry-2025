package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.Autons;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems
  private List<Subsystem> m_allSubsystems = new ArrayList<>();
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final Coral m_coral = Coral.getInstance();
  private final Algae m_algae = Algae.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Climber m_climber = new Climber();


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
   
   /*add code for addressable LED lights which will be LIT this year
      //assume we will use PWM port 9 with Andy Mark LED 12 V lights
      m_led = new AddressableLED(9);

      //Reuse buffer, default to length of 145 LEDs based on 16.4 feet of LEDs with 30 LEDs per meter
      m_ledBuffer = new AddressableLEDBuffer(100);
      m_led.setLength(m_ledBuffer.getLength());

      //set the data
      //m_led.setData(m_ledBuffer);
      m_led.start();

      //set LEDs to all one color assume yellow for this code
      for(var i = 0; i < m_ledBuffer.getLength(); i=i+2){
        m_ledBuffer.setRGB(i, 255, 255, 0);
        m_ledBuffer.setRGB(i+1,0,0,255);
      }
       m_led.setData(m_ledBuffer);

         
      System.out.println(lightBeamSensor.get()); */
   
       
     }

    /*  public void LEDPeriodic(){
    boolean isBlocked  = lightBeamSensor.get();
    if(isAutoLED) {
        if(isBlocked){
            //handle auto sensor blocked
            return;
        }
        // handle auto sensor not blocked
        return;
    }

    //handle teleop beam

     if(lightBeamSensor.get() == false){
        if(lightBeamState == 0){
          for(var i = 0; i < 100; i=i+1){
          m_ledBuffer.setRGB(i, 0, 255, 0);
          }
          m_led.setData(m_ledBuffer);
          //System.out.println(lightBeamSensor.get());
        }   
        lightBeamState = 1;           
      }
      else if(lightBeamSensor.get() == true){
        if(lightBeamState == 1){
          for(var i = 0; i < 100; i=i+1){
            m_ledBuffer.setRGB(i, 255, 255, 0);
          }
          m_led.setData(m_ledBuffer);
        }
        lightBeamState = 0;
      }
}

public boolean LEDBeamBreak(){
    return lightBeamSensor.get();
}*/

//public void setIsAuto(boolean _isAuto){
//  isAutoLED = _isAuto;
//}

     

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Grab selected auto from SmartDashboard drop down menu
    String selectedAutoName = SmartDashboard.getString("Auto Selector", Autons.autoNames[0]);
    return Autons.getSelectedAuto(selectedAutoName, m_drive, m_coral, m_elevator);
  }
}