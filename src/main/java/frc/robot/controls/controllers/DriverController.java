package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

public class DriverController extends FilteredController {
  private String m_smartDashboardKey = "DriverInput/";

  public DriverController(int port) {
    super(port, false, false);
  }

  public DriverController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Axis

  private final double k_triggerActivationThreshold = 0.5;
  private double k_lastTriggerValue = 0.0;

 public double getLeftY(){
  return getRawAxis(1);
 }
 public double getLeftX(){
  return getRawAxis(0);
 }
 public double getRightX(){
  return getRawAxis(4);
 }
 // public boolean getWantsSpeedMode() {
 //   return this.getFilteredAxis(2) > k_triggerActivationThreshold;
 // }

 public boolean getWantsScoreCoral() {
  return this.getRawButton(4);  //changed out to Left Bumper on Driver 
 }

  public boolean getWantsGroundAlgae() {
    return this.getRawButton(6);  // this is Right Bumpter
  }

  public boolean getWantsScoreAlgae() {
    return this.getRawButton(2);  // this is the B button
  }
  
  public boolean getClimberUp(){
    return this.getRawButton(4);  //this is the Y button
  }

  public boolean getClimberStopUp(){
    return this.getRawButtonReleased(4);
  }

  public boolean getClimberDown(){
    return this.getRawButton(1);  //this is the A button
  }
  
  public boolean getClimberStopDown(){
    return this.getRawButtonReleased(1);
  }


  public void outputTelemetry() {
    //SmartDashboard.putNumber(m_smartDashboardKey + "Forward", getForwardAxis());
    //SmartDashboard.putNumber(m_smartDashboardKey + "Turn", getTurnAxis());
  }
}
