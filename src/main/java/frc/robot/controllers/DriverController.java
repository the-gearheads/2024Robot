package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    // controller.setRumble(RumbleType.kBothRumble, Math.abs(controller.getRightX()));
    return Controllers.deadband(-controller.getRightX());
  }

  public double getSpeedUpAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getLeftTriggerAxis());
  }
  
  public double getSlowDownAxis() {
    if(isNull()) return 0;
    return Controllers.deadband(controller.getRightTriggerAxis());
  }

  public Trigger getAlignBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRightBumper());
  }

  public Trigger getAutoShootBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getAButton());
  }

  public Trigger getShootBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getXButton());
  }

  public Trigger getFeedBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getBButton());
  }

  public Trigger enableVision() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getStartButton());
  }

  public Trigger getDisableVisionBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getRawButton(7));
  }


  // dpad is oh shit btns, up is reset post, left is reconfigure
  public Trigger getResetPoseBtn() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 0);
  }

  public Trigger reconfigureEverything() {
    if(isNull()) return emptyTrigger();
    return new Trigger(() -> controller.getPOV() == 270);
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kBothRumble, rumble);
  }

  public Command getRumbleCommand(double rumble, double seconds) {
    return new InstantCommand(() -> {setRumble(rumble);}).withTimeout(seconds).finallyDo(() -> {setRumble(0);});
  }
}
