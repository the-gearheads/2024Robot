package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

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
    return Controllers.deadband(-controller.getRightX());
  }
}
