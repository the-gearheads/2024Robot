package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverController {

  XboxController controller;

  public DriverController(int id) {
    if(id == -1) {
      this.controller = null;
      return;
    }
    this.controller = new XboxController(id);
  }

  private boolean isNull() {
    return controller == null;
  }

  private Trigger emptyTrigger() {
    return new Trigger(() -> false);
  }

  public double getTranslateXAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftY());
  }

  public double getTranslateYAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(-controller.getLeftX());
  }

  public double getRotateAxis() {
    if(isNull()) return 0;
    // setRumble(Math.abs(controller.getRightX()));
    return Controllers.deadband(-controller.getRightX());
  }

  // dpad is oh shit btns, down is reset post, left is reconfigure
  public Trigger getResetPoseBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 180);
  }

  public Trigger reconfigureEverything() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 270);
  }

  public Trigger allowClimberOverride() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 90);
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kBothRumble, rumble);
  }

  public Command getRumbleCommand(double rumble, double seconds) {
    return Commands.runEnd(()->setRumble(rumble), ()->setRumble(0)).withTimeout(seconds);
  }

  public Command getRumbleCommand(double rumble, double seconds, int pulses) {
    final int pulseCounter[] = {0};
    return Commands.runEnd(()->setRumble(rumble), ()->{setRumble(0);pulseCounter[0]++;}).withTimeout(seconds).andThen(new WaitCommand(seconds*1.2)).repeatedly().until(() -> pulseCounter[0]==pulses);
  }
}
