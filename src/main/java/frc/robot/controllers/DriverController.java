package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
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
    return new Trigger(() -> controller.getBButton());
  }
}
