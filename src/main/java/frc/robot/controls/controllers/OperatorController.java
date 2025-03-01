package frc.robot.controls.controllers;

public class OperatorController extends FilteredController {

  public OperatorController(int port) {
    super(port, false, false);
  }

  public OperatorController(int port, boolean useDeadband, boolean useSquaredInput) {
    super(port, useDeadband, useSquaredInput);
  }

  // Axis
  private final double k_triggerActivationThreshold = 0.5;

  // CORAL
  public boolean getWantsStopCoral() {
    return this.getRawButton(5);  //Left Bumper
  }

  // ELEVATOR
  public boolean getWantsElevatorReset() {
    return this.getRawButton(7);  // Start button
  }

  public boolean getWantsElevatorStow() {
    return this.getRawButton(1);  //A button
  }

  public boolean getWantsElevatorL2() {
    return this.getRawButton(3);  //X button
  }

  public boolean getWantsElevatorL3() {
    return this.getRawButton(2);  //B button
  }

  public boolean getWantsElevatorL4() {
    return this.getRawButton(4);  //Y button
  }

  public boolean getWantsScoreCoral() {
    return this.getRawButton(6);  //Right Bumper
   }

  public boolean getWantsA1() {
    return this.getHatDown();
  }

  public boolean getWantsA2() {
    return this.getHatUp();
  }

  public boolean getWantsGroundAlgae() {
    return this.getHatLeft();
  }

  public boolean getWantsStopAlgae() {
    return this.getHatRight();
  }

  public double getMoveAlgaeUp(){
    return this.getRawAxis(3);
  }

 public double getMoveAlgaeDown(){
    return this.getRawAxis(4);
  }

}
