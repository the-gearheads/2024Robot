package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDriverController implements DriverController {

  XboxController controller;

  public XboxDriverController(int id) {
    this.controller = new XboxController(id);
  }

  @Override
  public double getTranslateXAxis() {
    return Controllers.deadband(-controller.getLeftY());
  }

  @Override
  public double getTranslateYAxis() {
    return Controllers.deadband(-controller.getLeftX());
  }

  @Override
  public double getRotateAxis() {
    controller.setRumble(RumbleType.kBothRumble, Math.abs(controller.getRightX()));
    return Controllers.deadband(-controller.getRightX());
  }

  @Override
  public double getSpeedModifierAxis() {
    return Controllers.deadband(controller.getRightTriggerAxis());
  }

  @Override
  public double getSlowModifierAxis() {
    return Controllers.deadband(controller.getLeftTriggerAxis());
  }

  @Override
  public Trigger getGyroZeroButton() {
    return new Trigger(()-> controller.getLeftBumper());
  }

    @Override
  public Trigger getResetPoseButton() {
    return new Trigger(()-> controller.getRightBumper());
  }

  @Override
  public Trigger getPatthfindButton() {
    return new Trigger(() -> controller.getXButton());
  }
}
